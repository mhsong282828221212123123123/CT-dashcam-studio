"""Train v5 CPU 3D-meter-loss contact model (14x14 FPN horizontal soft-argmax + bounded v residual).

Focuses purely on ego-relative 3D ground-contact position (no angle/direction head).
Selects strictly on validation 3D projected meter error; evaluates untouched test once.
No CUDA, no download, no estimated XYZ reused as targets.
"""
import argparse, copy, hashlib, json
from pathlib import Path
import numpy as np
from PIL import Image, ImageEnhance
import torch
from torch import nn
import torch.nn.functional as F
from torchvision.models import mobilenet_v3_small
from ct_scene.contact_learning import guard_splits, contact_target, metrics, CAMERAS
from ct_scene.surrounding import project_contact

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def to_tensor(im):
    arr = np.array(im, dtype=np.float32) / 255.0
    return ((arr - MEAN) / STD).transpose(2, 0, 1)


def u_crop_to_u_box(u_crop, rects, boxes):
    x = rects[:, 0] + u_crop * (rects[:, 2] - rects[:, 0])
    return (x - boxes[:, 0]) / (boxes[:, 2] - boxes[:, 0])


def project_uv_3d(uv, r, opt, proto):
    b = np.asarray(r['bbox_xyxy'], float)
    px = b[:2] + np.asarray(uv, float) * (b[2:] - b[:2])
    cam = r['camera']
    return project_contact(
        [px[0], px[1], px[0], px[1]],
        opt['cameras'][cam], proto['cameras'][cam],
        max_depth=140.0, minimum_ray_y=0.003,
    )


def contact_jacobian_m(uv_gt, r, opt, proto, eps=1e-3):
    """Exact local 2x2 Jacobian (meters per unit u_box, v_box) at gt contact."""
    p0 = project_uv_3d(uv_gt, r, opt, proto)[:2]
    pu = project_uv_3d([uv_gt[0] + eps, uv_gt[1]], r, opt, proto)[:2]
    pv = project_uv_3d([uv_gt[0], uv_gt[1] + eps], r, opt, proto)[:2]
    J = np.column_stack([(pu - p0) / eps, (pv - p0) / eps])
    # Clamp extreme distant vertical sensitivity to keep gradients stable
    norm_v = np.linalg.norm(J[:, 1])
    if norm_v > 120.0:
        J[:, 1] *= (120.0 / norm_v)
    return J.astype(np.float32)


def ground_3d_metrics(pred_uv, gt_uv, rows, opt, proto):
    errs = []; skipped = 0
    for pu, gu, r in zip(pred_uv, gt_uv, rows):
        try:
            pg = project_uv_3d(gu, r, opt, proto)[:2]
            pp = project_uv_3d(pu, r, opt, proto)[:2]
            errs.append(float(np.linalg.norm(pp - pg)))
        except Exception:
            skipped += 1
    arr = np.asarray(errs, float)
    return dict(
        samples=len(arr),
        skipped_out_of_range=skipped,
        mean_error_m=float(np.mean(arr)),
        median_error_m=float(np.median(arr)),
        p90_error_m=float(np.quantile(arr, 0.9)),
    )


class Contact3DMeterNet(nn.Module):
    """14x14 FPN horizontal soft-argmax u_crop + tightly bounded v_box residual."""
    def __init__(self, stage1, stage2):
        super().__init__()
        self.stage1 = stage1
        self.stage2 = stage2
        self.lat1 = nn.Conv2d(48, 48, 1)
        self.lat2 = nn.Conv2d(576, 48, 1)
        self.u_head = nn.Sequential(
            nn.Conv2d(48, 48, 3, padding=1),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.Dropout2d(0.2),
            nn.Conv2d(48, 1, 1),
        )
        self.v_head = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
            nn.Linear(48, 24),
            nn.ReLU(inplace=True),
            nn.Dropout(0.25),
            nn.Linear(24, 1),
        )
        g = torch.linspace(0.5 / 14.0, 1.0 - 0.5 / 14.0, 14)
        _, gx = torch.meshgrid(g, g, indexing='ij')
        self.register_buffer('grid_x', gx.reshape(1, 196))

    def forward(self, x):
        f14 = self.stage1(x)
        f7 = self.stage2(f14)
        fused = self.lat1(f14) + F.interpolate(self.lat2(f7), size=(14, 14), mode='bilinear', align_corners=False)
        probs = F.softmax(self.u_head(fused).flatten(1), dim=1)
        u_crop = (probs * self.grid_x).sum(dim=1)
        # Tightly bounded vertical contact near bbox bottom (0.97 +/- 0.04)
        v_box = 0.97 + 0.04 * torch.tanh(self.v_head(fused).squeeze(1))
        return torch.stack([u_crop, v_box], dim=1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--annotations', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--epochs', type=int, default=65)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)

    torch.set_num_threads(4)
    torch.manual_seed(20261125)
    np.random.seed(20261125)

    proto = json.loads(Path('configs/surrounding-prototype.json').read_text())
    opt = json.loads(Path('configs/multicam-optical-candidates.json').read_text())
    records = json.loads(a.annotations.read_text())['records']
    guard_splits(records)

    imgs, c_box, visible, ratios, rects, boxes, jacs = [], [], [], [], [], [], []
    for r in records:
        imgs.append(Image.open(r['crop']).convert('RGB').resize((224, 224), Image.Resampling.BICUBIC))
        t = contact_target(r)
        b = np.array(r['bbox_xyxy'], np.float32)
        cr = np.array(r['crop_rect_xyxy'], np.float32)
        ratios.append(float((b[2] - b[0]) / (b[3] - b[1])))
        rects.append(cr); boxes.append(b)
        if t is not None:
            c_box.append(t.astype(np.float32))
            visible.append(True)
            try:
                jacs.append(contact_jacobian_m(t, r, opt, proto))
            except Exception:
                jacs.append(np.array([[3.0, 0.0], [0.0, 15.0]], np.float32))
        else:
            c_box.append(np.array([0.5, 0.97], np.float32))
            visible.append(False)
            jacs.append(np.zeros((2, 2), np.float32))

    c_box = np.array(c_box); visible = np.array(visible, bool)
    ratios = np.array(ratios, np.float32); rects = np.array(rects); boxes = np.array(boxes)
    jacs = np.array(jacs, np.float32)
    train = np.array([r['split'] == 'train' for r in records])
    validation = np.array([r['split'] == 'validation' for r in records])
    test = np.array([r['split'] == 'test' for r in records])

    state = torch.load(a.weights, map_location='cpu', weights_only=True)
    prefix = 'net.net.0.'
    bb_state = {k[len(prefix):]: v for k, v in state.items() if k.startswith(prefix)}
    feats = mobilenet_v3_small(weights=None).features
    feats.load_state_dict(bb_state, strict=True)
    for idx, ch in enumerate(feats):
        for param in ch.parameters():
            param.requires_grad = (idx >= 8)

    net = Contact3DMeterNet(nn.Sequential(*list(feats[:9])), nn.Sequential(*list(feats[9:])))
    bb_params = [param for n, param in net.named_parameters() if param.requires_grad and (n.startswith('stage1') or n.startswith('stage2'))]
    hd_params = [param for n, param in net.named_parameters() if param.requires_grad and not (n.startswith('stage1') or n.startswith('stage2'))]
    opt_t = torch.optim.AdamW([{'params': bb_params, 'lr': 2e-4}, {'params': hd_params, 'lr': 1e-3}], weight_decay=2e-2)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt_t, T_max=a.epochs)

    tr_idx = np.where(train & visible)[0]
    va_idx = np.where(validation & visible)[0]
    va_rows = [records[i] for i in va_idx]
    all_xo = torch.from_numpy(np.array([to_tensor(im) for im in imgs], np.float32))
    all_xf = torch.from_numpy(np.array([to_tensor(im.transpose(Image.Transpose.FLIP_LEFT_RIGHT)) for im in imgs], np.float32))

    # Precompute crop-normalized u_crop for training targets
    u_crop_gt = ((boxes[:, 0] + c_box[:, 0] * (boxes[:, 2] - boxes[:, 0])) - rects[:, 0]) / np.maximum(rects[:, 2] - rects[:, 0], 1.0)

    best_val = (999.0, 0, None)
    for ep in range(1, a.epochs + 1):
        net.train()
        perm = np.random.permutation(tr_idx)
        for s in range(0, len(perm), 16):
            batch_i = perm[s:s + 16]
            xs, y_ub, y_vb, j_list, r_list, cr_list, bx_list, sc_list, x0_list = [], [], [], [], [], [], [], [], []
            for i in batch_i:
                im = imgs[i]; uc = float(u_crop_gt[i]); vb = float(c_box[i, 1])
                J = jacs[i].copy(); cr = rects[i].copy(); bx = boxes[i].copy()
                flipped = False
                if np.random.rand() < 0.5:
                    im = im.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
                    uc = 1.0 - uc
                    flipped = True
                # Random horizontal/vertical sub-crop jitter
                x0_frac = 0.0; sc = 1.0
                if np.random.rand() < 0.65:
                    sc_try = float(np.random.uniform(0.86, 1.0))
                    nw = int(round(224 * sc_try)); nh = int(round(224 * sc_try))
                    x0 = np.random.randint(0, 224 - nw + 1); y0 = np.random.randint(0, 224 - nh + 1)
                    nu = (uc * 224.0 - x0) / nw
                    if 0.05 <= nu <= 0.95:
                        im = im.crop((x0, y0, x0 + nw, y0 + nh)).resize((224, 224), Image.Resampling.BICUBIC)
                        x0_frac = x0 / 224.0; sc = nw / 224.0
                if np.random.rand() < 0.75:
                    im = ImageEnhance.Brightness(im).enhance(np.random.uniform(0.65, 1.35))
                    im = ImageEnhance.Contrast(im).enhance(np.random.uniform(0.7, 1.3))
                    im = ImageEnhance.Color(im).enhance(np.random.uniform(0.75, 1.25))
                xs.append(to_tensor(im))
                # Map target u_box consistently with flip
                ub = (1.0 - float(c_box[i, 0])) if flipped else float(c_box[i, 0])
                if flipped:
                    J[:, 0] = -J[:, 0]
                    # Also mirror crop_rect & bbox x around 1280 for exact u_crop -> u_box mapping
                    cr = np.array([1280.0 - cr[2], cr[1], 1280.0 - cr[0], cr[3]], np.float32)
                    bx = np.array([1280.0 - bx[2], bx[1], 1280.0 - bx[0], bx[3]], np.float32)
                y_ub.append(ub); y_vb.append(vb); j_list.append(J); r_list.append(ratios[i])
                cr_list.append(cr); bx_list.append(bx); sc_list.append(sc); x0_list.append(x0_frac)

            bx_t = torch.from_numpy(np.array(xs, np.float32))
            pred = net(bx_t) # (B, 2) = [u_crop_aug, v_box]
            sc_t = torch.from_numpy(np.array(sc_list, np.float32))
            x0_t = torch.from_numpy(np.array(x0_list, np.float32))
            u_crop_orig = pred[:, 0] * sc_t + x0_t
            cr_t = torch.from_numpy(np.array(cr_list, np.float32))
            box_t = torch.from_numpy(np.array(bx_list, np.float32))
            u_box_pred = ((cr_t[:, 0] + u_crop_orig * (cr_t[:, 2] - cr_t[:, 0])) - box_t[:, 0]) / (box_t[:, 2] - box_t[:, 0])
            v_box_pred = pred[:, 1]
            du = u_box_pred - torch.from_numpy(np.array(y_ub, np.float32))
            dv = v_box_pred - torch.from_numpy(np.array(y_vb, np.float32))
            duv = torch.stack([du, dv], dim=1).unsqueeze(2) # (B, 2, 1)
            J_t = torch.from_numpy(np.array(j_list, np.float32)) # (B, 2, 2)
            dp_m = torch.bmm(J_t, duv).squeeze(2) # (B, 2) meter error vector
            dist_m = torch.sqrt((dp_m ** 2).sum(dim=1) + 1e-6)
            loss_3d = F.smooth_l1_loss(dist_m, torch.zeros_like(dist_m), beta=0.35)
            ar_t = torch.from_numpy(np.array(r_list, np.float32))
            loss_2d_u = F.smooth_l1_loss(du * ar_t, torch.zeros_like(du), beta=0.05)
            loss = loss_3d + 1.0 * loss_2d_u
            opt_t.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(net.parameters(), 2.0)
            opt_t.step()
        sched.step()
        if ep >= 20 and ep % 3 == 0:
            net.eval()
            with torch.inference_mode():
                po = net(all_xo[va_idx]).numpy()
                pf = net(all_xf[va_idx]).numpy()
                u_c = 0.5 * (po[:, 0] + (1.0 - pf[:, 0]))
                v_b = 0.5 * (po[:, 1] + pf[:, 1])
                uv_b = np.c_[u_crop_to_u_box(u_c, rects[va_idx], boxes[va_idx]), v_b]
            m3 = ground_3d_metrics(uv_b, c_box[va_idx], va_rows, opt, proto)
            # Select strictly on validation 3D ego-relative mean + p90 meter error
            sel_score = m3['mean_error_m'] + 0.15 * m3['p90_error_m']
            if sel_score < best_val[0]:
                best_val = (sel_score, ep, copy.deepcopy(net.state_dict()))
        if ep % 15 == 0:
            print(f'3D-MeterNet epoch {ep}/{a.epochs} best_val_3d_m={best_val[0]:.3f}m (ep {best_val[1]})', flush=True)

    net.load_state_dict(best_val[2])
    net.eval()

    with torch.inference_mode():
        po_all = net(all_xo).numpy()
        pf_all = net(all_xf).numpy()
        u_c_all = 0.5 * (po_all[:, 0] + (1.0 - pf_all[:, 0]))
        v_b_all = 0.5 * (po_all[:, 1] + pf_all[:, 1])
        contact_pred = np.c_[u_crop_to_u_box(u_c_all, rects, boxes), v_b_all]

    onnx_path = a.output / 'contact-encoder.onnx'
    torch.onnx.export(
        net, torch.zeros(1, 3, 224, 224), onnx_path,
        input_names=['rgb_normalized'], output_names=['contact_uv_hybrid'],
        dynamic_axes={'rgb_normalized': {0: 'batch'}, 'contact_uv_hybrid': {0: 'batch'}},
        opset_version=17,
    )

    predictions = []
    for i, r in enumerate(records):
        predictions.append(dict(
            id=r['id'], split=r['split'], camera=r['camera'],
            contact_uv=contact_pred[i].tolist(),
            target_contact_uv=contact_target(r).tolist() if visible[i] else None,
            direction='disabled', target_direction=r['direction'],
        ))

    results = {}
    for split, mask in [('train', train), ('validation', validation), ('test', test)]:
        chosen = mask & visible; rows = [r for r, k in zip(records, chosen) if k]
        baseline = np.tile([0.5, 1.0], (int(sum(chosen)), 1))
        counts = {c: sum(r['camera'] == c for r in rows) for c in CAMERAS}
        results[split] = dict(
            contact_2d_model=metrics(contact_pred[chosen], c_box[chosen], rows),
            contact_2d_baseline=metrics(baseline, c_box[chosen], rows),
            projected_3d_ground_m_model=ground_3d_metrics(contact_pred[chosen], c_box[chosen], rows, opt, proto),
            projected_3d_ground_m_baseline=ground_3d_metrics(baseline, c_box[chosen], rows, opt, proto),
            contact_count_by_camera=counts,
        )

    tr = results['test']
    c2d_pass = (
        tr['contact_2d_model']['mean_error_vehicle_height'] < 0.9 * tr['contact_2d_baseline']['mean_error_vehicle_height']
        and tr['contact_2d_model']['p90_error_vehicle_height'] <= tr['contact_2d_baseline']['p90_error_vehicle_height']
    )
    c3d_pass = (
        tr['projected_3d_ground_m_model']['mean_error_m'] < 0.9 * tr['projected_3d_ground_m_baseline']['mean_error_m']
        and tr['projected_3d_ground_m_model']['p90_error_m'] <= tr['projected_3d_ground_m_baseline']['p90_error_m']
    )
    report = dict(
        status='trained_experimental_3d_ground_contact_head',
        adapter_version='v5',
        adapter_config=dict(contact_mode='fpn14_u_crop_bounded_v_box', flip_tta=True, direction_enabled=False),
        runtime_adopted=False,
        metric_xyz_accuracy='PROTOTYPE_RIG_GROUND_PROJECTION_ONLY_NOT_SURVEYED_XYZ',
        new_supervised_weights=True,
        full_backbone_finetuned=False,
        partial_backbone_finetuned='mobilenetv3_features_8_to_12',
        source_split_unit='recorded drive date/session',
        samples={s: sum(r['split'] == s for r in records) for s in ['train', 'validation', 'test']},
        selected=dict(contact=dict(architecture='14x14_fpn_u_soft_argmax_plus_bounded_v_residual', unfreeze_from=8, best_epoch=best_val[1], validation_3d_mean_m=best_val[0])),
        results=results,
        contact_2d_candidate_pass=bool(c2d_pass),
        projected_3d_ground_m_candidate_pass=bool(c3d_pass),
        all_cameras_have_three_test_contacts=all(n >= 3 for n in tr['contact_count_by_camera'].values()),
        annotation_provenance=dict(
            reviewers=sorted({r.get('reviewer', 'unspecified') for r in records}),
            independent_human_review=json.loads(a.annotations.read_text()).get('independent_human_review', False),
            source='visual image tire-pair labels; projected with prototype camera rig (no LiDAR/GPS ground truth)',
        ),
        adoption_rule='Keep disabled until camera rig calibration and multi-frame 3D track continuity are separately validated',
        cuda=False,
        annotations_sha256=hashlib.sha256(a.annotations.read_bytes()).hexdigest(),
        encoder_sha256=hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
    )
    np.savez_compressed(a.output / 'features.npz', ids=np.array([r['id'] for r in records]), features=po_all)
    np.savez_compressed(a.output / 'adapter.npz', v_center=np.array([0.97], np.float32), v_half_range=np.array([0.04], np.float32))
    (a.output / 'provenance.json').write_text(json.dumps(dict(
        source_weights=str(a.weights),
        source_weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),
        annotations_sha256=report['annotations_sha256'],
        encoder_version='v5',
        torch=torch.__version__,
        device='cpu',
        cuda_used=False,
        backbone='KITTI mobilenet_v3_small partial fine-tune (>=8) + 14x14 FPN u_crop + bounded v_box; trained with prototype 3D ground meter Jacobian loss',
        target='ego-relative 3D ground-projected tire-contact position (no angle head)',
    ), indent=2))
    (a.output / 'report.json').write_text(json.dumps(report, indent=2))
    (a.output / 'predictions.json').write_text(json.dumps(predictions, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
