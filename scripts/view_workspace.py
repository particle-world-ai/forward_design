"""Display the latest dual-arm FK result in Isaac Sim, with the timeline stopped."""
import argparse
import asyncio
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--results-dir', type=Path)
parser.add_argument('--test-frames', type=int, default=0)
parser.add_argument('--screenshot', type=Path)
parser.add_argument('--ik-latest', action='store_true', help='Open the most recent full IK verification')
parser.add_argument('--wait-for-results', type=float, default=0, help='Wait for calculation output while Isaac Sim initializes')
parser.add_argument('--cycle-ik', action='store_true', help='Show successive solved configurations, kinematically, without physics/path validation')
parser.add_argument('--pose-seconds', type=float, default=4.)
args = parser.parse_args()
pointer = ROOT/('ik_runs/latest_success.json' if args.ik_latest else 'runs/latest_success.json')
out = args.results_dir or Path(json.loads(pointer.read_text())['run_dir'])
scene = out/'workspace_preview.usda'
if args.pose_seconds <= 0 or args.wait_for_results < 0:
    parser.error('Invalid interval or wait timeout')
if not scene.is_file() and not args.wait_for_results:
    raise FileNotFoundError('Run run_fk.bat first: '+str(scene))
from isaacsim import SimulationApp
app = SimulationApp({'headless': False, 'width': 1440, 'height': 1000})
code = 0
try:
    import omni.timeline
    from isaacsim.core.utils.stage import open_stage, get_current_stage, is_stage_loading
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import UsdGeom
    deadline = time.monotonic()+args.wait_for_results
    while not (scene.is_file() and (out/'summary.json').is_file()):
        if not app.is_running() or time.monotonic() > deadline:
            raise TimeoutError('Calculation results not ready: '+str(out))
        app.update()
    if not open_stage(str(scene)):
        raise RuntimeError('Unable to open '+str(scene))
    while is_stage_loading():
        app.update()
    omni.timeline.get_timeline_interface().stop()
    stage = get_current_stage()
    summary = json.loads((out/'summary.json').read_text(encoding='utf-8'))
    unit = summary['stage_meters_per_unit']
    bounds = [arm['world_bounds_m'] for arm in summary['arms'].values()]
    center = [(min(b['min'][i] for b in bounds)+max(b['max'][i] for b in bounds))/2/unit for i in range(3)]
    size = max(max(b['max'][i] for b in bounds)-min(b['min'][i] for b in bounds) for i in range(3))/unit
    band = summary.get('height_filter')
    if band:
        xy = band['xy_polygon_m']
        center = [(min(p[0] for p in xy)+max(p[0] for p in xy))/2/unit,
                  (min(p[1] for p in xy)+max(p[1] for p in xy))/2/unit+.3/unit,
                  (band['table_top_world_z_m']+.4)/unit]
        size = max(max(p[i] for p in xy)-min(p[i] for p in xy) for i in range(2))/unit
    eye = [center[0]+size*1.1, center[1]-size*1.5, center[2]+size*.9]
    set_camera_view(eye=eye, target=center)
    counts = {str(p.GetPath()): len(UsdGeom.Points(p).GetPointsAttr().Get()) for p in stage.Traverse() if p.IsA(UsdGeom.Points)}
    visible_counts = {str(p.GetPath()): len(UsdGeom.Points(p).GetPointsAttr().Get()) for p in stage.Traverse()
                      if p.IsA(UsdGeom.Points) and UsdGeom.Imageable(p).ComputeVisibility() != UsdGeom.Tokens.invisible}
    print('VIEWER_READY '+json.dumps(counts), flush=True)
    demo = {}
    if args.cycle_ik:
        import numpy as np
        from pxr import Gf, Usd
        from dual_fk import ArmFK, matrix
        raw = Usd.Stage.Open(str(Path(summary['ik_source_run'])/'input_scene.usdc'))
        for side in ['left', 'right']:
            with np.load(out/(side+'_ik_solved.npz')) as z:
                data = {key:z[key] for key in ['sample_id','arm_q','pose_world']}
            if not len(data['sample_id']):
                continue
            spec = summary['arms'][side]
            model = ArmFK(raw, summary['robot_root'], spec['base_prim'], spec['tip_prim'])
            demo[side] = (model, data, np.linspace(0,len(data['sample_id'])-1,min(24,len(data['sample_id'])),dtype=int))
            marker = '/IKStudy/'+side.capitalize()+'TargetAxes'
            UsdGeom.Xform.Define(stage,marker)
            for axis,color in enumerate([(1.,.05,.05),(.05,1.,.05),(.05,.2,1.)]):
                curves = UsdGeom.BasisCurves.Define(stage,marker+'/'+['X','Y','Z'][axis])
                endpoint = [0.,0.,0.]; endpoint[axis]=.06/unit
                curves.CreateTypeAttr('linear'); curves.CreateCurveVertexCountsAttr([2])
                curves.CreatePointsAttr([Gf.Vec3f(0.,0.,0.),Gf.Vec3f(*endpoint)])
                curves.CreateWidthsAttr([.003/unit]); curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
                curves.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        print('IK_KINEMATIC_DISPLAY: discrete solved configurations; no physical motion or collision validation.',flush=True)
    demo_step, next_demo = 0, 0.
    frames, captured = 0, False
    while app.is_running():
        app.update()
        frames += 1
        if demo and time.monotonic() >= next_demo:
            for side,(model,data,indices) in demo.items():
                index=int(indices[demo_step%len(indices)])
                _,links=model.fk(data['arm_q'][index:index+1],return_links=True)
                for child,t in list(links.items())[1:]:
                    prim=stage.GetPrimAtPath(child);local=np.linalg.inv(matrix(prim.GetParent()))@t[0]
                    xf=UsdGeom.Xformable(prim);xf.MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))
                target=data['pose_world'][index].copy();target[:3,3]/=unit
                marker=UsdGeom.Xformable(stage.GetPrimAtPath('/IKStudy/'+side.capitalize()+'TargetAxes'))
                marker.MakeMatrixXform().Set(Gf.Matrix4d(target.T.tolist()))
            demo_step+=1;next_demo=time.monotonic()+args.pose_seconds
        if args.screenshot and not captured and frames >= (min(args.test_frames, 120) if args.test_frames else 120):
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if args.screenshot.exists():
                args.screenshot.unlink()
            capture = capture_viewport_to_file(get_active_viewport(), str(args.screenshot))
            task = asyncio.ensure_future(capture.wait_for_result())
            while app.is_running() and not task.done():
                app.update()
            task.result()
            # The viewport future may finish before its asynchronous file writer.
            deadline = time.monotonic()+10
            while app.is_running() and time.monotonic() < deadline:
                if args.screenshot.is_file() and args.screenshot.stat().st_size > 0:
                    break
                app.update()
            captured = True
        if args.test_frames and frames >= args.test_frames:
            break
        if frames == 120:
            (out/'viewer_status.json').write_text(json.dumps(dict(ready=True, frames=frames, visible_point_counts=visible_counts,
                height_filter=band, timeline='stopped', ik_configuration_cycling=bool(demo),
                screenshot=str(args.screenshot) if captured else None), indent=2), encoding='utf-8')
            print('VIEWER_RENDERED', flush=True)
    if args.test_frames:
        if frames < args.test_frames or (args.screenshot and not args.screenshot.is_file()):
            raise RuntimeError('Rendering test did not complete')
        (out/'viewer_validation.json').write_text(json.dumps(dict(passed=True, frames=frames, point_counts=counts,
            timeline='stopped', screenshot=str(args.screenshot) if args.screenshot else None), indent=2), encoding='utf-8')
        print('VIEWER_TEST_PASSED', flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    code = 1
finally:
    app.close(exit_code=code)
