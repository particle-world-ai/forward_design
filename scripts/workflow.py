"""USD + height band -> validated dual-arm FK CSV, HTML and Isaac Sim preview."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def resolve_prims(stage, args):
    from pxr import Usd, UsdGeom
    paths = list(stage.Traverse())
    robot = args.robot_root
    if not robot:
        bases = {}
        for side in ('left', 'right'):
            found = [p for p in paths if p.GetName().startswith(side+'_') and p.GetName().endswith('_link0')]
            if len(found) != 1:
                raise ValueError(f'Ambiguous {side} base: {[str(p.GetPath()) for p in found]}; provide --robot-root and --{side}-base/--{side}-tip')
            bases[side] = found[0].GetPath()
        robot = str(bases['left'].GetCommonPrefix(bases['right']))
        if robot in ('/', '/World'):
            raise ValueError('No unique common robot root; provide --robot-root')
        # Prefer the named robot assembly ancestor when present, preserving its coordinate frame.
        for ancestor in bases['left'].GetPrefixes():
            if ancestor.name == 'franka_vision_and_manipulation_kit':
                robot = str(ancestor)
                break
    if not stage.GetPrimAtPath(robot) or not stage.GetPrimAtPath(robot).IsActive():
        raise ValueError(f'Missing/disabled robot: {robot}')
    table = args.table_prim
    if not table:
        known = '/World/airbag_switch_baked/node_1_桌面/Cube_002'
        p = stage.GetPrimAtPath(known)
        if p and p.IsActive() and p.IsA(UsdGeom.Mesh):
            table = known
        else:
            candidates = [str(p.GetPath()) for p in paths if p.IsA(UsdGeom.Mesh)
                          and any(x in str(p.GetPath()).lower() for x in ('桌面', 'tabletop', 'table_top'))]
            if len(candidates) != 1:
                raise ValueError(f'Table Mesh is ambiguous: {candidates}; specify --table-prim')
            table = candidates[0]
    return robot, table


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--min-height', required=True, type=float, help='metres above table, inclusive')
    parser.add_argument('--max-height', required=True, type=float, help='metres above table, inclusive')
    parser.add_argument('--samples', type=int, default=1048576, help='per arm; preferably a power of 2')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--robot-root')
    parser.add_argument('--table-prim')
    for side in ('left', 'right'):
        parser.add_argument('--'+side+'-base')
        parser.add_argument('--'+side+'-tip')
    parser.add_argument('--height-only', action='store_true')
    parser.add_argument('--voxel-m', type=float, default=.02)
    parser.add_argument('--output-root', type=Path, default=Path.cwd()/'fk_workspace_runs')
    parser.add_argument('--no-view', action='store_true', help='still export USD and offline HTML')
    parser.add_argument('--viewer-test-frames', type=int, default=0)
    args = parser.parse_args()
    import numpy as np
    from pxr import Usd, UsdGeom
    from dual_fk import discover, table_filter
    if not (np.isfinite([args.min_height,args.max_height,args.voxel_m]).all()
            and 0 <= args.min_height <= args.max_height and args.voxel_m > 0 and args.samples > 0):
        parser.error('Require finite 0 <= min-height <= max-height, samples > 0 and voxel-m > 0')
    scene = args.scene.resolve(strict=True)
    stage = Usd.Stage.Open(str(scene))
    if not stage:
        raise ValueError('Cannot open USD')
    robot, table = resolve_prims(stage, args)
    cfg = dict(scene=str(scene), robot_root=robot, table_prim=table,
        arms={s:{k+'_prim':getattr(args,s+'_'+k) for k in ('base','tip') if getattr(args,s+'_'+k)} for s in ('left','right')},
        samples=args.samples, seed=args.seed, voxel_m=args.voxel_m, joint_margin_rad=.00001,
        point_width_m=.006, min_height_m=args.min_height, max_height_m=args.max_height,
        filter_above_table=True, table_xy_only=not args.height_only, html_max_points_per_arm=20000)
    table_filter(stage, UsdGeom.GetStageMetersPerUnit(stage), cfg)
    for side in ('left','right'):
        spec=cfg['arms'][side]
        model=discover(stage,robot,side,spec.get('base_prim'),spec.get('tip_prim'))
        spec.update(base_prim=model.base_path,tip_prim=model.tip_path)
    job = args.output_root.resolve()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    job.mkdir(parents=True)
    config = job/'request.json'
    config.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    out = job/'results'
    print('JOB_DIRECTORY',job,flush=True)
    subprocess.run([sys.executable,str(HERE/'dual_fk.py'),'--config',str(config),'--out',str(out)],check=True)
    subprocess.run([sys.executable,str(HERE/'verify_results.py'),'--results-dir',str(out)],check=True)
    report = json.loads((out/'summary.json').read_text(encoding='utf-8'))
    print('FILTERED_COUNTS',json.dumps({s:a['selected_count'] for s,a in report['arms'].items()}),flush=True)
    print('OFFLINE_VISUALIZATION',out/'workspace.html',flush=True)
    (args.output_root.resolve()/'latest_success.json').write_text(json.dumps({'run_dir':str(out)},indent=2),encoding='utf-8')
    if not args.no_view:
        command=[sys.executable,'-u',str(HERE/'view_workspace.py'),'--results-dir',str(out),'--screenshot',str(out/'workspace.png')]
        if args.viewer_test_frames:
            command += ['--test-frames',str(args.viewer_test_frames)]
        with (out/'viewer.log').open('w',encoding='utf-8') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        (out/'viewer_process.json').write_text(json.dumps({'pid':child.pid,'log':str(out/'viewer.log'),'command':command},indent=2),encoding='utf-8')
        print('VIEWER_STARTING',child.pid,'Read viewer_status.json for rendered confirmation.',flush=True)


if __name__=='__main__':
    main()
