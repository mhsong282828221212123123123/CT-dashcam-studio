"""Train v4 CPU supervised 14x14 FPN spatial soft-argmax contact + frozen-KITTI direction model.

Selects strictly on validation; evaluates untouched test once. No CUDA, no download, no metric XYZ targets.
"""
import argparse, copy, hashlib, json
from pathlib import Path
import numpy as np
from PIL import Image, ImageEnhance
import torch
from torch import nn
import torch.nn.functional as F
from torchvision.models import mobilenet_v3_small
from ct_scene.contact_learning import (
    guard_splits, contact_target, metadata, flip_record,
    flip_direction_index, ridge, metrics, CAMERAS, DIRECTIONS
)

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def to_tensor(im):
    arr = np.array(im, dtype=np.float32) / 255.0
    return ((arr - MEAN) / STD).transpose(2, 0, 1)


def crop_uv_to_box_uv(uv_crop, rects, boxes):
    xy = rects[:, :2] + uv_crop * (rects[:, 2:] - rects[:, :2])
    return (xy - boxes[:, :2]) / (boxes[:, 2:] - boxes[:, :2])


class ContactFpnHead(nn.Module):
    def __init__(self, stage1, stage2):
        super().__init__()
        self.stage1 = stage1
        self.stage2 = stage2
        self.lat1 = nn.Conv2d(48, 48, 1)
        self.lat2 = nn.Conv2d(576, 48, 1)
        self.head = nn.Sequential(
            nn.Conv2d(48, 48, 3, padding=1),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.Dropout2d(0.2),
            nn.Conv2d(48, 1, 1),
        )
        g = torch.linspace(0.5 / 14.0, 1.0 - 0.5 / 14.0, 14)
        gy, gx = torch.meshgrid(g, g, indexing='ij')
        self.register_buffer('grid_x', gx.reshape(1, 196))
        self.register_buffer('grid_y', gy.reshape(1, 196))

    def forward(self, x):
        f14 = self.stage1(x)
        f7 = self.stage2(f14)
        fused = self.lat1(f14) + F.interpolate(self.lat2(f7), size=(14, 14), mode='bilinear', align_corners=False)
        probs = F.softmax(self.head(fused).flatten(1), dim=1)
        return torch.stack([(probs * self.grid_x).sum(1), (probs * self.grid_y).sum(1)], dim=1)


class UnifiedV4Onnx(nn.Module):
    def __init__(self, contact_net, dir_backbone):
        super().__init__()
        self.contact_net = contact_net
        self.dir_backbone = dir_backbone
        self.dir_pool = nn.AvgPool2d(4, stride=3)

    def forward(self, x):
        uv_crop = self.contact_net(x)
        dir_features = self.dir_pool(self.dir_backbone(x)).flatten(1)
        return uv_crop, dir_features


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--annotations', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--epochs', type=int, default=60)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)

    torch.set_num_threads(4)
    torch.manual_seed(20261125)
    np.random.seed(20261125)

    records = json.loads(a.annotations.read_text())['records']
    guard_splits(records)

    imgs, c_crop, c_box, visible, direction, ratios, rects, boxes = [], [], [], [], [], [], [], []
    for r in records:
        imgs.append(Image.open(r['crop']).convert('RGB').resize((224, 224), Image.Resampling.BICUBIC))
        t = contact_target(r)
        b = np.array(r['bbox_xyxy'], np.float32)
        cr = np.array(r['crop_rect_xyxy'], np.float32)
        ratios.append(float((b[2] - b[0]) / (b[3] - b[1])))
        rects.append(cr); boxes.append(b)
        if t is not None:
            xy = b[:2] + t * (b[2:] - b[:2])
            c_crop.append(((xy - cr[:2]) / np.maximum(cr[2:] - cr[:2], 1.0)).astype(np.float32))
            c_box.append(t.astype(np.float32))
            visible.append(True)
        else:
            c_crop.append(np.array([0.5, 0.85], np.float32))
            c_box.append(np.array([0.5, 1.0], np.float32))
            visible.append(False)
        direction.append(DIRECTIONS.index(r['direction']) if r['direction'] in DIRECTIONS else -1)

    c_crop = np.array(c_crop); c_box = np.array(c_box); visible = np.array(visible, bool)
    direction = np.array(direction, np.int64); ratios = np.array(ratios, np.float32)
    rects = np.array(rects); boxes = np.array(boxes)
    train = np.array([r['split'] == 'train' for r in records])
    validation = np.array([r['split'] == 'validation' for r in records])
    test = np.array([r['split'] == 'test' for r in records])

    state = torch.load(a.weights, map_location='cpu', weights_only=True)
    prefix = 'net.net.0.'
    bb_state = {k[len(prefix):]: v for k, v in state.items() if k.startswith(prefix)}

    # 1. Train 14x14 FPN spatial soft-argmax contact net strictly on train, selecting on validation
    c_feats = mobilenet_v3_small(weights=None).features
    c_feats.load_state_dict(bb_state, strict=True)
    for idx, ch in enumerate(c_feats):
        for param in ch.parameters():
            param.requires_grad = (idx >= 8)
    contact_net = ContactFpnHead(nn.Sequential(*list(c_feats[:9])), nn.Sequential(*list(c_feats[9:])))
    bb_params = [param for n, param in contact_net.named_parameters() if param.requires_grad and (n.startswith('stage1') or n.startswith('stage2'))]
    hd_params = [param for n, param in contact_net.named_parameters() if param.requires_grad and not (n.startswith('stage1') or n.startswith('stage2'))]
    opt = torch.optim.AdamW([{'params': bb_params, 'lr': 2e-4}, {'params': hd_params, 'lr': 1e-3}], weight_decay=2e-2)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=a.epochs)

    tr_c_idx = np.where(train & visible)[0]
    va_c_idx = np.where(validation & visible)[0]
    va_c_rows = [records[i] for i in va_c_idx]
    all_xo = torch.from_numpy(np.array([to_tensor(im) for im in imgs], np.float32))
    all_xf = torch.from_numpy(np.array([to_tensor(im.transpose(Image.Transpose.FLIP_LEFT_RIGHT)) for im in imgs], np.float32))

    best_val = (999.0, 0, None)
    for ep in range(1, a.epochs + 1):
        contact_net.train()
        perm = np.random.permutation(tr_c_idx)
        for s in range(0, len(perm), 16):
            batch_i = perm[s:s + 16]
            xs, ys, rs = [], [], []
            for i in batch_i:
                im = imgs[i]; uv = c_crop[i].copy()
                if np.random.rand() < 0.5:
                    im = im.transpose(Image.Transpose.FLIP_LEFT_RIGHT); uv[0] = 1.0 - uv[0]
                if np.random.rand() < 0.7:
                    sc = np.random.uniform(0.84, 1.0)
                    nw = int(round(224 * sc)); nh = int(round(224 * sc))
                    x0 = np.random.randint(0, 224 - nw + 1); y0 = np.random.randint(0, 224 - nh + 1)
                    nu = (uv[0] * 224.0 - x0) / nw; nv = (uv[1] * 224.0 - y0) / nh
                    if 0.05 <= nu <= 0.95 and 0.15 <= nv <= 0.98:
                        im = im.crop((x0, y0, x0 + nw, y0 + nh)).resize((224, 224), Image.Resampling.BICUBIC)
                        uv = np.array([nu, nv], np.float32)
                if np.random.rand() < 0.75:
                    im = ImageEnhance.Brightness(im).enhance(np.random.uniform(0.65, 1.35))
                    im = ImageEnhance.Contrast(im).enhance(np.random.uniform(0.7, 1.3))
                    im = ImageEnhance.Color(im).enhance(np.random.uniform(0.75, 1.25))
                xs.append(to_tensor(im)); ys.append(uv); rs.append(ratios[i])
            bx = torch.from_numpy(np.array(xs, np.float32))
            by = torch.from_numpy(np.array(ys, np.float32))
            br = torch.from_numpy(np.array(rs, np.float32))
            opt.zero_grad()
            pred = contact_net(bx)
            diff = pred - by
            wdiff = torch.stack([diff[:, 0] * br, diff[:, 1]], dim=1)
            loss = F.smooth_l1_loss(wdiff, torch.zeros_like(wdiff), beta=0.05)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(contact_net.parameters(), 2.0)
            opt.step()
        sched.step()
        if ep >= 20 and ep % 3 == 0:
            contact_net.eval()
            with torch.inference_mode():
                uo = contact_net(all_xo[va_c_idx])
                uf = contact_net(all_xf[va_c_idx])
                uv_c = 0.5 * (uo + torch.stack([1.0 - uf[:, 0], uf[:, 1]], dim=1)).numpy()
            uv_b = crop_uv_to_box_uv(uv_c, rects[va_c_idx], boxes[va_c_idx])
            score = metrics(uv_b, c_box[va_c_idx], va_c_rows)['mean_error_vehicle_height']
            if score < best_val[0]:
                best_val = (score, ep, copy.deepcopy(contact_net.state_dict()))
        if ep % 15 == 0:
            print(f'Contact FPN epoch {ep}/{a.epochs} best_val={best_val[0]:.4f} (ep {best_val[1]})', flush=True)

    contact_net.load_state_dict(best_val[2])
    contact_net.eval()

    # 2. Frozen KITTI backbone for direction + unified ONNX export
    dir_bb = mobilenet_v3_small(weights=None).features
    dir_bb.load_state_dict(bb_state, strict=True)
    dir_bb.eval()
    unified = UnifiedV4Onnx(contact_net, dir_bb).eval()

    with torch.inference_mode():
        uo_all, fo_all = unified(all_xo)
        uf_all, ff_all = unified(all_xf)
        uv_crop_pred = 0.5 * (uo_all + torch.stack([1.0 - uf_all[:, 0], uf_all[:, 1]], dim=1)).numpy()
        contact_pred = crop_uv_to_box_uv(uv_crop_pred, rects, boxes)
        fo = fo_all.numpy().astype(float); ff = ff_all.numpy().astype(float)

    train_f = np.vstack([fo[train], ff[train]])
    mean = train_f.mean(0); scale = np.maximum(train_f.std(0), 0.1)
    zdo = np.clip((fo - mean) / scale, -6, 6) / np.sqrt(fo.shape[1])
    zdf = np.clip((ff - mean) / scale, -6, 6) / np.sqrt(ff.shape[1])
    mdo = np.array([metadata(r) for r in records])
    mdf = np.array([metadata(flip_record(r)) for r in records])

    tr_d = train & (direction >= 0); va_d = validation & (direction >= 0)
    yd = np.vstack([np.eye(4)[direction[tr_d]], np.eye(4)[flip_direction_index(direction[tr_d])]])
    candidates = []
    for strength in [0.0, 0.25, 0.5, 1.0, 2.0]:
        xo = np.c_[zdo * strength, mdo]; xf = np.c_[zdf * strength, mdf]; xd = np.vstack([xo[tr_d], xf[tr_d]])
        for alpha in [0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0, 25.0]:
            w, b = ridge(xd, yd, alpha)
            po = xo[va_d] @ w + b; pf = (xf[va_d] @ w + b)[:, [0, 1, 3, 2]]; pred = 0.5 * (po + pf)
            score = float(np.mean((pred - np.eye(4)[direction[va_d]]) ** 2))
            candidates.append((score, strength, alpha, w, b))
    d_score, d_strength, d_alpha, dw, db = min(candidates, key=lambda r: r[0])
    po = np.c_[zdo * d_strength, mdo] @ dw + db; pf = (np.c_[zdf * d_strength, mdf] @ dw + db)[:, [0, 1, 3, 2]]
    direction_pred = np.argmax(0.5 * (po + pf), axis=1)

    # Export unified CPU ONNX
    onnx_path = a.output / 'contact-encoder.onnx'
    torch.onnx.export(
        unified, torch.zeros(1, 3, 224, 224), onnx_path,
        input_names=['rgb_normalized'], output_names=['contact_uv_crop', 'dir_features'],
        dynamic_axes={'rgb_normalized': {0: 'batch'}, 'contact_uv_crop': {0: 'batch'}, 'dir_features': {0: 'batch'}},
        opset_version=17,
    )

    # Evaluate train, validation, and ONCE on untouched test
    selected = dict(
        contact=dict(architecture='14x14_fpn_spatial_soft_argmax', unfreeze_from=8, best_epoch=best_val[1], validation_selection_score=best_val[0]),
        direction=dict(feature_strength=d_strength, alpha=d_alpha, validation_selection_score=d_score),
    )
    predictions = []
    for i, r in enumerate(records):
        predictions.append(dict(
            id=r['id'], split=r['split'], camera=r['camera'],
            contact_uv=contact_pred[i].tolist(),
            target_contact_uv=contact_target(r).tolist() if visible[i] else None,
            direction=DIRECTIONS[direction_pred[i]], target_direction=r['direction'],
        ))
    results = {}
    for split, mask in [('train', train), ('validation', validation), ('test', test)]:
        chosen = mask & visible; rows = [r for r, k in zip(records, chosen) if k]
        baseline = np.tile([0.5, 1.0], (int(sum(chosen)), 1))
        counts = {c: sum(r['camera'] == c for r in rows) for c in CAMERAS}
        known = mask & (direction >= 0)
        majority = np.bincount(direction[train & (direction >= 0)], minlength=4).argmax()
        results[split] = dict(
            contact_model=metrics(contact_pred[chosen], c_box[chosen], rows),
            bbox_bottom_baseline=metrics(baseline, c_box[chosen], rows),
            contact_count_by_camera=counts,
            direction_samples=int(sum(known)),
            direction_accuracy=float(np.mean(direction_pred[known] == direction[known])),
            training_majority_baseline=float(np.mean(majority == direction[known])),
        )
    tr = results['test']
    candidate = (
        tr['contact_model']['mean_error_vehicle_height'] < 0.9 * tr['bbox_bottom_baseline']['mean_error_vehicle_height']
        and tr['contact_model']['p90_error_vehicle_height'] <= tr['bbox_bottom_baseline']['p90_error_vehicle_height']
    )
    report = dict(
        status='trained_experimental_image_heads',
        adapter_version='v4',
        adapter_config=dict(contact_mode='fpn14_soft_argmax_crop_uv', flip_tta=True, dir_feature_dim=int(fo.shape[1])),
        runtime_adopted=False,
        metric_xyz_accuracy='NOT_EVALUATED',
        new_supervised_weights=True,
        full_backbone_finetuned=False,
        partial_backbone_finetuned='contact_branch_mobilenetv3_features_8_to_12',
        source_split_unit='recorded drive date/session',
        samples={s: sum(r['split'] == s for r in records) for s in ['train', 'validation', 'test']},
        selected=selected,
        results=results,
        contact_2d_candidate_pass=bool(candidate),
        all_cameras_have_three_test_contacts=all(n >= 3 for n in tr['contact_count_by_camera'].values()),
        direction_candidate_pass=tr['direction_accuracy'] >= 0.7,
        annotation_provenance=dict(
            reviewers=sorted({r.get('reviewer', 'unspecified') for r in records}),
            independent_human_review=json.loads(a.annotations.read_text()).get('independent_human_review', False),
            source='visual image labels; no surveyed XYZ',
        ),
        adoption_rule='Keep disabled: 2D contact and coarse image direction do not validate metric 3D centre or world yaw',
        cuda=False,
        annotations_sha256=hashlib.sha256(a.annotations.read_bytes()).hexdigest(),
        encoder_sha256=hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
    )
    np.savez_compressed(a.output / 'features.npz', ids=np.array([r['id'] for r in records]), contact_uv_crop=uo_all.numpy(), features=fo)
    np.savez_compressed(a.output / 'adapter.npz', feature_mean=mean, feature_scale=scale, direction_w=dw, direction_b=db)
    (a.output / 'provenance.json').write_text(json.dumps(dict(
        source_weights=str(a.weights),
        source_weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),
        annotations_sha256=report['annotations_sha256'],
        encoder_version='v4',
        torch=torch.__version__,
        device='cpu',
        cuda_used=False,
        backbone='KITTI mobilenet_v3_small; contact branch partial fine-tune (>=8) + 14x14 FPN soft-argmax; direction branch frozen',
        target='image contacts only, not metric XYZ',
    ), indent=2))
    (a.output / 'report.json').write_text(json.dumps(report, indent=2))
    (a.output / 'predictions.json').write_text(json.dumps(predictions, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
