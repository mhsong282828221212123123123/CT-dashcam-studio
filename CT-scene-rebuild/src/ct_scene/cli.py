import argparse
import json
from pathlib import Path
from .artifacts import verify


def main():
    parser = argparse.ArgumentParser(description='Independent CT Scene stage tools')
    commands = parser.add_subparsers(dest='command', required=True)
    check = commands.add_parser('verify-artifact')
    check.add_argument('directory')
    timing = commands.add_parser('audit-pts')
    timing.add_argument('--front', required=True)
    timing.add_argument('--output', required=True)
    timing.add_argument('--duration', type=float, default=35)
    sei = commands.add_parser('audit-sei')
    sei.add_argument('--source', required=True)
    sei.add_argument('--output', required=True)
    sei.add_argument('--duration', type=float, default=35)
    inv = commands.add_parser('inventory')
    inv.add_argument('--roots', nargs='+', required=True)
    inv.add_argument('--legacy-results', required=True)
    inv.add_argument('--output', required=True)
    ann = commands.add_parser('prepare-annotations')
    ann.add_argument('--front', required=True)
    ann.add_argument('--output', required=True)
    ann.add_argument('--duration', type=float, default=35)
    ann.add_argument('--sample-hz', type=float, default=5)
    val = commands.add_parser('validate-annotations')
    val.add_argument('--package', required=True)
    val.add_argument('--annotations', required=True)
    assist=commands.add_parser('propose-labels')
    assist.add_argument('--package',required=True)
    assist.add_argument('--references',required=True)
    assist.add_argument('--output',required=True)
    masks=commands.add_parser('raw-masks')
    masks.add_argument('--package',required=True)
    masks.add_argument('--model',required=True)
    masks.add_argument('--output',required=True)
    masks.add_argument('--backend',choices=['directml','cpu'],default='directml')
    features=commands.add_parser('static-features')
    features.add_argument('--package',required=True)
    features.add_argument('--masks',required=True)
    features.add_argument('--output',required=True)
    motion=commands.add_parser('motion-candidates')
    motion.add_argument('--features',required=True)
    motion.add_argument('--lens-candidate',required=True)
    motion.add_argument('--camera',default='front')
    motion.add_argument('--output',required=True)
    sfm=commands.add_parser('optical-sfm')
    sfm.add_argument('--package',required=True)
    sfm.add_argument('--masks',required=True)
    sfm.add_argument('--output',required=True)
    sfm.add_argument('--camera',default='front')
    sfm.add_argument('--holdout-times',nargs='+',type=int,default=[4,12,20])
    sfm.add_argument('--sample-hz',type=int,choices=[1,2,5],default=1)
    sfm.add_argument('--bootstrap-artifact')
    sfm.add_argument('--keep-optics-fixed',action='store_true')
    sfm.add_argument('--roi',choices=['legacy_conservative','camera_aware_v1'],default='legacy_conservative')
    scale=commands.add_parser('wheel-scale-check')
    scale.add_argument('--sfm',required=True)
    scale.add_argument('--telemetry',required=True)
    scale.add_argument('--output',required=True)
    multi=commands.add_parser('multicam-connectivity')
    affine=commands.add_parser('affine-connectivity-probe')
    for command in (multi,affine):
        command.add_argument('--package',required=True)
        command.add_argument('--masks',required=True)
        command.add_argument('--lens-config',required=True)
        command.add_argument('--output',required=True)
    affine.add_argument('--side-delays',nargs='+',type=int,default=[2,4,6,8])
    affine.add_argument('--front-times',nargs='+',type=int,default=list(range(0,32,4)))
    affine.add_argument('--roi',choices=['camera_aware_v1','legacy_conservative'],default='camera_aware_v1')
    manual=commands.add_parser('manual-planar-check')
    plane=commands.add_parser('planar-warp-probe')
    for command in (manual,plane):
        command.add_argument('--package',required=True)
        command.add_argument('--masks',required=True)
        command.add_argument('--lens-config',required=True)
        command.add_argument('--output',required=True)
    manual.add_argument('--reference',required=True)
    plane.add_argument('--probe',required=True)
    multi_view=commands.add_parser('manual-multiview-probe')
    multi_view.add_argument('--package',required=True)
    multi_view.add_argument('--masks',required=True)
    multi_view.add_argument('--sfm',required=True)
    multi_view.add_argument('--lens-config',required=True)
    multi_view.add_argument('--reference',required=True)
    multi_view.add_argument('--output',required=True)
    plane_pose=commands.add_parser('plane-pose-probe')
    plane_pose.add_argument('--package',required=True)
    plane_pose.add_argument('--tracks',required=True)
    plane_pose.add_argument('--lens-config',required=True)
    plane_pose.add_argument('--output',required=True)
    plane_pose.add_argument('--refine-target-focal',action='store_true')
    transfer=commands.add_parser('side-transfer-probe')
    transfer.add_argument('--package',required=True)
    transfer.add_argument('--tracks',required=True)
    transfer.add_argument('--plane',required=True)
    transfer.add_argument('--output',required=True)
    transfer.add_argument('--duration',type=float,default=2)
    transfer.add_argument('--raw-source')
    transfer.add_argument('--direction',type=int,choices=[-1,1],default=1)
    tracker=commands.add_parser('camera-tracks')
    tracker.add_argument('--package',required=True)
    tracker.add_argument('--detections',required=True)
    tracker.add_argument('--output',required=True)
    review=commands.add_parser('tracking-review')
    review.add_argument('--package',required=True)
    review.add_argument('--tracks',required=True)
    review.add_argument('--output',required=True)
    review.add_argument('--sparse-references')
    review.add_argument('--presence-references')
    proto=commands.add_parser('single-lead-prototype')
    proto.add_argument('--package',required=True)
    proto.add_argument('--tracks',required=True)
    proto.add_argument('--telemetry',required=True)
    proto.add_argument('--lens-config',required=True)
    proto.add_argument('--ego-model',required=True)
    proto.add_argument('--actor-model',required=True)
    proto.add_argument('--output',required=True)
    proto.add_argument('--camera-height',type=float,default=1.3)
    surround=commands.add_parser('surrounding-prototype')
    for name in ('package','tracks','base','lens-config','prototype-config','assets-root','output'):
        surround.add_argument('--'+name,required=True)
    surround.add_argument('--road-masks')
    surround.add_argument('--osm-context')
    surround.add_argument('--preview-times',nargs='+',type=float)
    surround.add_argument('--telemetry')
    surround.add_argument('--image-headings',help='Verified front image body-heading evidence artifact')
    surround.add_argument('--contact-adapter',help='Opt-in experimental CPU learned ground-contact adapter directory')
    manual=commands.add_parser('manual-render',help='Separate human-assisted actor correction copy')
    for name in ('scene','base','corrections','assets-root','prototype-config','output'):
        manual.add_argument('--'+name,required=True)
    replay=commands.add_parser('replay-scene',help='Render frozen world states without rerunning inference')
    for name in ('scene','assets-root','prototype-config','output'):replay.add_argument('--'+name,required=True)
    full=commands.add_parser('full-source-timeline')
    full.add_argument('--assets-root')
    full.add_argument('--ego-rotation',help='Verified static-background own-turn evidence')
    for name in ('package','tracks','telemetry','lens-config','source','actor-model','output'):
        full.add_argument('--'+name,required=True)
    osm=commands.add_parser('osm-context')
    osm.add_argument('--telemetry',required=True)
    osm.add_argument('--osm',required=True)
    osm.add_argument('--output',required=True)
    roads=commands.add_parser('road-masks')
    for name in ('package','model','output'):roads.add_argument('--'+name,required=True)
    roads.add_argument('--backend',choices=['directml','cpu'],default='directml')
    osm_fetch=commands.add_parser('osm-fetch')
    osm_fetch.add_argument('--telemetry',required=True)
    osm_fetch.add_argument('--cache',required=True)
    numerical=commands.add_parser('review-export')
    numerical.add_argument('--result',required=True)
    numerical.add_argument('--output',required=True)
    posacc=commands.add_parser('position-accuracy',help='Evaluate 3D contact-label accuracy and scene position surrogates')
    posacc.add_argument('--output',required=True)
    posacc.add_argument('--annotations')
    posacc.add_argument('--lens-config',default='configs/multicam-optical-candidates.json')
    posacc.add_argument('--prototype-config',default='configs/surrounding-prototype.json')
    posacc.add_argument('--adapters',nargs='*',default=[])
    posacc.add_argument('--scenes',nargs='*',default=[])
    pipeline=commands.add_parser('analyze')
    for name in ('front','output','detector','road-model','lens-config','prototype-config','assets-root'):pipeline.add_argument('--'+name,required=True)
    pipeline.add_argument('--osm')
    pipeline.add_argument('--backend',choices=['directml','cpu'],default='directml')
    preflight=commands.add_parser('input-preflight',help='Check full four-camera and driving-data coverage before inference')
    for name in ('front','telemetry','output'):preflight.add_argument('--'+name,required=True)
    args = parser.parse_args()
    try:
        if args.command == 'input-preflight':
            from .input_preflight import run,require_supported
            manifest=run(args.front,args.telemetry,args.output)
            require_supported(args.output)
        elif args.command == 'analyze':
            from .pipeline import analyze
            state=analyze(args.front,args.output,args.detector,args.road_model,args.lens_config,args.prototype_config,args.assets_root,args.osm,args.backend)
            print(json.dumps(state));return 0
        elif args.command == 'review-export':
            from .quality_review import review
            result=review(args.result,args.output)
            print(json.dumps({k:v for k,v in result.items() if k not in ('failures','worst_motion_events')}));return 0 if result['numerical_gate']=='PASS' else 2
        elif args.command == 'position-accuracy':
            from .position_accuracy import run
            result=run(args.output,args.annotations,args.lens_config,args.prototype_config,args.adapters,args.scenes)
            print(json.dumps({'status':'OK','scenes':list(result['scene_surrogates'].keys()),'has_contact_benchmark':result['contact_label_benchmark'] is not None}));return 0
        elif args.command == 'osm-fetch':
            from .osm_context import fetch
            print(json.dumps(fetch(args.telemetry,args.cache)));return 0
        elif args.command == 'road-masks':
            from .road_perception import run
            manifest=run(args.package,args.model,args.output,args.backend)
        elif args.command == 'osm-context':
            from .osm_context import run
            manifest=run(args.telemetry,args.osm,args.output)
        elif args.command == 'audit-pts':
            from .ingest import audit_pts
            manifest = audit_pts(args.front, args.output, args.duration)
        elif args.command == 'audit-sei':
            from .telemetry import audit_sei
            manifest = audit_sei(args.source, args.output, args.duration)
        elif args.command == 'inventory':
            from .catalog import inventory
            manifest = inventory(args.roots, args.legacy_results, args.output)
        elif args.command == 'prepare-annotations':
            from .annotation import prepare
            manifest = prepare(args.front, args.output, args.duration, args.sample_hz)
        elif args.command == 'validate-annotations':
            from .annotation import validate_annotations
            print(json.dumps(validate_annotations(args.package, args.annotations)))
            return 0
        elif args.command == 'propose-labels':
            from .label_assist import propose
            manifest=propose(args.package,args.references,args.output)
        elif args.command == 'raw-masks':
            from .mask_prepass import run
            manifest=run(args.package,args.model,args.output,args.backend)
        elif args.command == 'static-features':
            from .static_features import run
            manifest=run(args.package,args.masks,args.output)
        elif args.command == 'motion-candidates':
            from .motion_candidates import run
            manifest=run(args.features,args.lens_candidate,args.output,args.camera)
        elif args.command == 'optical-sfm':
            from .sfm import run
            manifest=run(args.package,args.masks,args.output,args.camera,tuple(args.holdout_times),args.bootstrap_artifact,args.sample_hz,not args.keep_optics_fixed,args.roi)
        elif args.command == 'wheel-scale-check':
            from .wheel_scale import run
            manifest=run(args.sfm,args.telemetry,args.output)
        elif args.command == 'multicam-connectivity':
            from .multicam_sfm import run
            manifest=run(args.package,args.masks,args.lens_config,args.output)
        elif args.command == 'affine-connectivity-probe':
            from .affine_probe import run
            manifest=run(args.package,args.masks,args.lens_config,args.output,tuple(args.side_delays),tuple(args.front_times),args.roi)
        elif args.command == 'manual-planar-check':
            from .landmark_diagnostics import run
            manifest=run(args.package,args.masks,args.lens_config,args.reference,args.output)
        elif args.command == 'planar-warp-probe':
            from .planar_warp_probe import run
            manifest=run(args.package,args.masks,args.probe,args.lens_config,args.output)
        elif args.command == 'manual-multiview-probe':
            from .manual_rig_probe import run
            manifest=run(args.package,args.masks,args.sfm,args.lens_config,args.reference,args.output)
        elif args.command == 'plane-pose-probe':
            from .plane_pose_probe import run
            manifest=run(args.package,args.tracks,args.lens_config,args.output,args.refine_target_focal)
        elif args.command == 'side-transfer-probe':
            from .side_transfer_probe import run
            manifest=run(args.package,args.tracks,args.plane,args.output,args.duration,args.raw_source,args.direction)
        elif args.command == 'camera-tracks':
            from .tracking import run
            manifest=run(args.package,args.detections,args.output)
        elif args.command == 'tracking-review':
            from .tracking_review import run
            manifest=run(args.package,args.tracks,args.output,args.sparse_references,args.presence_references)
        elif args.command == 'single-lead-prototype':
            from .prototype_front import run
            manifest=run(args.package,args.tracks,args.telemetry,args.lens_config,args.ego_model,args.actor_model,args.output,args.camera_height)
        elif args.command == 'manual-render':
            from .manual_review import render
            manifest=render(args.scene,args.base,args.corrections,args.assets_root,args.prototype_config,args.output)
        elif args.command == 'replay-scene':
            from .scene_replay import run
            manifest=run(args.scene,args.assets_root,args.prototype_config,args.output)
        elif args.command == 'surrounding-prototype':
            from .surrounding import run
            assets={k:str(Path(args.assets_root)/('tesla_fsd_'+name+'.glb')) for k,name in [('ego','sedan'),('van','van'),('suv','suv'),('sedan','sedan'),('truck','small_truck'),('box_truck','box_truck'),('bus','bus')]}
            manifest=run(args.package,args.tracks,args.base,args.lens_config,args.prototype_config,assets,args.output,args.road_masks,args.osm_context,args.preview_times,args.telemetry,args.image_headings,contact_adapter=args.contact_adapter)
        elif args.command == 'full-source-timeline':
            from .full_timeline import run
            manifest=run(args.package,args.tracks,args.telemetry,args.lens_config,args.source,args.actor_model,args.output,args.assets_root,args.ego_rotation)
        else:
            manifest = verify(args.directory)
    except (ValueError, OSError, KeyError) as error:
        print(json.dumps({'integrity': 'FAIL', 'reason': str(error)}))
        return 2
    print(json.dumps({'integrity': 'PASS', 'artifact_id': manifest['artifact_id'],
                      'quality_gate': manifest['quality_gate']}))
    return 0
