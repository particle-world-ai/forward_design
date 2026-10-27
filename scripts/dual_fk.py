"""Joint-limit FK sampling directly from a dual Franka USD. No IK or collision solver."""
import argparse
import csv
import datetime
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from scipy.spatial import ConvexHull
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

ROOT = Path(__file__).resolve().parent


def matrix(prim):
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T


def rigid(t, name):
    if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or np.linalg.det(t[:3, :3]) < 0:
        raise ValueError(f'Non-rigid/scaled frame {name}: apply scale to the robot asset before sampling')
    return t


def frame(joint, side):
    t = np.eye(4)
    p = getattr(joint, f'GetLocalPos{side}Attr')().Get()
    q = getattr(joint, f'GetLocalRot{side}Attr')().Get()
    t[:3, :3] = Rotation.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()
    t[:3, 3] = p
    return t


class ArmFK:
    def __init__(self, stage, robot, base, tip):
        self.base_path, self.tip_path = base, tip
        self.base = rigid(matrix(stage.GetPrimAtPath(base)), base)
        incoming = {}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot)):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if not joint.GetJointEnabledAttr().Get():
                continue
            parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if parents and children:
                child = str(children[0])
                if child in incoming:
                    raise ValueError('Multiple incoming joints: ' + child)
                incoming[child] = (joint, str(parents[0]))
        # A flange/TCP may be a plain fixed Xform under the last rigid body.
        tip_prim = stage.GetPrimAtPath(tip)
        if not tip_prim:
            raise ValueError('Missing end frame: ' + tip)
        body = tip_prim
        while str(body.GetPath()) not in incoming and str(body.GetPath()) != base:
            if body.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError('Disconnected rigid body at tip: ' + str(body.GetPath()))
            body = body.GetParent()
            if not body or not body.GetPath().HasPrefix(Sdf.Path(robot)):
                raise ValueError('No connected body for tip: ' + tip)
        self.tip_offset = rigid(np.linalg.inv(matrix(body)) @ matrix(tip_prim), tip)
        chain, seen = [], set()
        link = str(body.GetPath())
        while link != base:
            if link in seen or link not in incoming:
                raise ValueError('Broken/cyclic chain: ' + link)
            seen.add(link)
            joint, parent = incoming[link]
            chain.append((joint, parent, link))
            link = parent
        self.chain, self.names, lower, upper = [], [], [], []
        self.authored_q = []
        self.authored_joint_error = 0.
        for joint, parent, child in reversed(chain):
            prim = joint.GetPrim()
            index, axis = None, None
            a, b = frame(joint, 0), np.linalg.inv(frame(joint, 1))
            delta = np.linalg.inv(matrix(stage.GetPrimAtPath(parent)) @ a) @ matrix(stage.GetPrimAtPath(child)) @ np.linalg.inv(b)
            if prim.IsA(UsdPhysics.RevoluteJoint):
                rev = UsdPhysics.RevoluteJoint(prim)
                index = len(self.names)
                self.names.append(prim.GetName())
                axis = np.eye(3)['XYZ'.index(rev.GetAxisAttr().Get())]
                lower.append(np.deg2rad(rev.GetLowerLimitAttr().Get()))
                upper.append(np.deg2rad(rev.GetUpperLimitAttr().Get()))
                q = float(Rotation.from_matrix(delta[:3, :3]).as_rotvec() @ axis)
                self.authored_q.append(q)
                expected = np.eye(4)
                expected[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
            elif prim.IsA(UsdPhysics.FixedJoint):
                expected = np.eye(4)
            else:
                raise ValueError('Unsupported joint: ' + str(prim.GetPath()))
            self.authored_joint_error = max(self.authored_joint_error, float(np.max(abs(delta - expected))))
            self.chain.append((a, b, index, axis, parent, child))
        self.lower, self.upper = np.array(lower), np.array(upper)
        if len(self.names) != 7 or not np.all(np.isfinite([self.lower, self.upper])) or np.any(self.lower >= self.upper):
            raise ValueError('Expected seven finite bounded revolute joints: ' + str(self.names))
        self.authored_q = np.array(self.authored_q)
        self.authored_tip_error = float(np.max(abs(self.fk(self.authored_q[None])[0] - matrix(tip_prim))))
        if max(self.authored_tip_error, self.authored_joint_error) > 2e-5:
            raise ValueError(f'USD joint frames do not match authored geometry: {tip}, error={self.authored_tip_error}')

    def fk(self, q, return_links=False):
        t = np.broadcast_to(self.base, (len(q), 4, 4)).copy()
        links = {self.base_path: t.copy()}
        for a, b, index, axis, parent, child in self.chain:
            t = t @ a
            if index is not None:
                r = np.broadcast_to(np.eye(4), t.shape).copy()
                r[:, :3, :3] = Rotation.from_rotvec(q[:, index, None] * axis).as_matrix()
                t = t @ r
            t = t @ b
            if return_links:
                links[child] = t.copy()
        return (t @ self.tip_offset, links) if return_links else t @ self.tip_offset


def discover(stage, robot, side, base_override=None, tip_override=None):
    prims = list(Usd.PrimRange(stage.GetPrimAtPath(robot)))
    def unique(suffix):
        matches = [str(p.GetPath()) for p in prims if p.GetName().startswith(side + '_') and p.GetName().endswith(suffix)]
        if len(matches) != 1:
            raise ValueError(f'Expected one {side} *{suffix}; found {matches}. Set the frame in config.json.')
        return matches[0]
    return ArmFK(stage, robot, base_override or unique('_link0'), tip_override or unique('_link8'))


def metres(poses, unit):
    result = poses.copy()
    result[..., :3, 3] *= unit
    return result


def save_csv(path, q, poses, sample_ids=None):
    fields = ['sample_id'] + [f'joint_{i+1}_rad' for i in range(7)]
    arrays = [(np.arange(len(q)) if sample_ids is None else sample_ids)[:, None], q]
    for name, t in poses.items():
        fields += [f'{name}_{x}_m' for x in 'xyz'] + [f'{name}_q{x}' for x in 'xyzw']
        arrays += [t[:, :3, 3], Rotation.from_matrix(t[:, :3, :3]).as_quat() if len(t) else np.empty((0, 4))]
    data = np.column_stack(arrays)
    np.savetxt(path, data, delimiter=',', header=','.join(fields), comments='',
               fmt=['%d'] + ['%.12g'] * (data.shape[1]-1), encoding='utf-8')


def add_points(stage, path, points_m, unit, color, width_m, ids=None):
    points = UsdGeom.Points.Define(stage, path)
    xyz = (points_m/unit).astype(np.float32)
    points.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(xyz))
    points.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.full(len(xyz), width_m/unit, dtype=np.float32)))
    points.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if ids is not None:
        points.CreateIdsAttr(Vt.Int64Array.FromNumpy(np.asarray(ids, dtype=np.int64)))
    if len(xyz):
        points.CreateExtentAttr([Gf.Vec3f(*map(float, xyz.min(0))), Gf.Vec3f(*map(float, xyz.max(0)))])


def table_filter(stage, unit, cfg):
    if not cfg.get('filter_above_table', False):
        return None
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError('The table-height filter currently requires a Z-up stage')
    prim = stage.GetPrimAtPath(cfg['table_prim'])
    if not prim or not prim.IsA(UsdGeom.Mesh):
        raise ValueError('table_prim must point to a valid table Mesh')
    vertices = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
    t = matrix(prim)
    xyz = (vertices @ t[:3, :3].T + t[:3, 3])*unit
    hull = ConvexHull(xyz[:, :2])
    top = float(xyz[:, 2].max())
    return dict(table_prim=cfg['table_prim'], table_top_world_z_m=top,
        min_height_m=cfg['min_height_m'], max_height_m=cfg['max_height_m'],
        world_z_min_m=top+cfg['min_height_m'], world_z_max_m=top+cfg['max_height_m'],
        table_xy_only=cfg.get('table_xy_only', True), xy_polygon_m=xyz[hull.vertices, :2].tolist(),
        xy_halfspaces=hull.equations.tolist(),
        definition='Flange origin world-Z height above the highest table mesh vertex; optional convex XY table footprint. Not whole gripper/object clearance.')


def height_mask(poses, band):
    if band is None:
        return np.ones(len(poses), dtype=bool)
    p = poses[:, :3, 3]
    mask = (p[:, 2] >= band['world_z_min_m']-1e-10) & (p[:, 2] <= band['world_z_max_m']+1e-10)
    if band['table_xy_only']:
        eq = np.asarray(band['xy_halfspaces'])
        mask &= np.all(p[:, :2] @ eq[:, :2].T + eq[:, 2] <= 1e-10, axis=1)
    return mask


def preview(stage, out, models, samples, poses, selected, overlap, unit, cfg, band):
    scene = out/'input_scene.usdc'
    stage.Flatten().Export(str(scene))
    view = Usd.Stage.CreateNew(str(out/'workspace_preview.usda'))
    view.GetRootLayer().subLayerPaths = ['./input_scene.usdc']
    UsdGeom.SetStageMetersPerUnit(view, unit)
    UsdGeom.SetStageUpAxis(view, UsdGeom.GetStageUpAxis(stage))
    UsdGeom.Xform.Define(view, '/DualWorkspace')
    colors = {'left': (.1, .55, 1.), 'right': (1., .35, .08)}
    for side, model in models.items():
        # Display a sampled configuration. This is a static FK illustration, not a collision-cleared pose.
        display_id = int(selected[side][0]) if len(selected[side]) else 0
        _, links = model.fk(samples[side][display_id:display_id+1], return_links=True)
        for child, values in list(links.items())[1:]:
            prim = view.GetPrimAtPath(child)
            parent_world = matrix(prim.GetParent())
            local = np.linalg.inv(parent_world) @ values[0]
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTransformOp(opSuffix='fkPreview').Set(Gf.Matrix4d(local.T.tolist()))
        ids = selected[side]
        add_points(view, '/DualWorkspace/'+side.capitalize()+'FK', poses[side][ids, :3, 3], unit, colors[side], cfg['point_width_m'], ids)
        all_path = '/DualWorkspace/'+side.capitalize()+'FKAll'
        add_points(view, all_path, poses[side][:, :3, 3], unit, colors[side], cfg['point_width_m'], np.arange(len(samples[side])))
        UsdGeom.Imageable(view.GetPrimAtPath(all_path)).MakeInvisible()
    if band is not None:
        xy = np.asarray(band['xy_polygon_m'])
        xy = np.vstack([xy, xy[0]])
        for name, z in [('HeightBandLower', band['world_z_min_m']), ('HeightBandUpper', band['world_z_max_m'])]:
            curves = UsdGeom.BasisCurves.Define(view, '/DualWorkspace/'+name)
            points = np.column_stack([xy, np.full(len(xy), z)])/unit
            curves.CreateTypeAttr('linear')
            curves.CreateCurveVertexCountsAttr([len(points)])
            curves.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
            curves.CreateWidthsAttr([.002/unit])
            curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
            curves.CreateDisplayColorAttr([Gf.Vec3f(.1, .9, .65)])
    add_points(view, '/DualWorkspace/SharedPositionVoxels', overlap, unit, (.25, 1., .25), cfg['voxel_m']*.45)
    view.GetRootLayer().Save()


def html_view(out, poses, selected, overlap, models, unit, max_points, band):
    import plotly.graph_objects as go
    fig = go.Figure()
    for side, color in [('left', '#248fff'), ('right', '#ff6925')]:
        available = selected[side]
        ids = available[np.linspace(0, len(available)-1, min(max_points, len(available)), dtype=int)]
        p = poses[side][ids, :3, 3]
        fig.add_trace(go.Scatter3d(x=p[:, 0], y=p[:, 1], z=p[:, 2], mode='markers', name=side+' FK',
            marker=dict(size=3, color=color, opacity=.8), customdata=ids,
            hovertemplate='sample_id=%{customdata}<br>x=%{x:.3f} m<br>y=%{y:.3f} m<br>z=%{z:.3f} m<extra>'+side+'</extra>'))
        p0 = models[side].base[:3, 3]*unit
        fig.add_trace(go.Scatter3d(x=[p0[0]], y=[p0[1]], z=[p0[2]], mode='markers+text', text=[side+' base'],
                                 marker=dict(size=7, color=color), name=side+' base'))
    p = overlap
    fig.add_trace(go.Scatter3d(x=p[:, 0], y=p[:, 1], z=p[:, 2], mode='markers', name='Shared position voxels (approx.)',
                             visible='legendonly', marker=dict(size=2, color='#29bc46')))
    if band is not None:
        xy = np.asarray(band['xy_polygon_m']); xy = np.vstack([xy, xy[0]])
        for z in [band['world_z_min_m'], band['world_z_max_m']]:
            fig.add_trace(go.Scatter3d(x=xy[:, 0], y=xy[:, 1], z=np.full(len(xy), z), mode='lines',
                name=f'Band Z={z:.4f} m', line=dict(color='#25b69b', width=3)))
    title = 'Dual Franka: flange FK (no IK/collision test)'
    if band is not None:
        title += f" — {band['min_height_m']*100:g}–{band['max_height_m']*100:g} cm above table"
    fig.update_layout(title=title,
                      scene=dict(aspectmode='data', xaxis_title='World X (m)', yaxis_title='World Y (m)', zaxis_title='World Z (m)'),
                      margin=dict(l=0, r=0, b=0, t=45))
    fig.write_html(out/'workspace.html', include_plotlyjs=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config.json')
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--samples', type=int, help='Sample count PER ARM')
    parser.add_argument('--seed', type=int)
    parser.add_argument('--voxel-m', type=float)
    parser.add_argument('--min-height', type=float, dest='min_height_m', help='Flange height above table, metres')
    parser.add_argument('--max-height', type=float, dest='max_height_m', help='Flange height above table, metres')
    parser.add_argument('--table-prim')
    parser.add_argument('--no-height-filter', action='store_true')
    parser.add_argument('--height-only', action='store_true', help='Do not restrict XY to the table projection')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    for key in ['scene', 'samples', 'seed', 'voxel_m', 'min_height_m', 'max_height_m', 'table_prim']:
        if getattr(args, key) is not None:
            cfg[key] = str(getattr(args, key)) if key == 'scene' else getattr(args, key)
    if args.min_height_m is not None or args.max_height_m is not None:
        cfg['filter_above_table'] = True
    if args.no_height_filter:
        cfg['filter_above_table'] = False
    if args.height_only:
        cfg['table_xy_only'] = False
    if cfg.get('filter_above_table', False) and not (np.isfinite(cfg['min_height_m']) and np.isfinite(cfg['max_height_m']) and 0 <= cfg['min_height_m'] <= cfg['max_height_m']):
        parser.error('Require finite 0 <= min-height <= max-height')
    scene = Path(cfg['scene'])
    if not scene.is_absolute():
        scene = args.config.resolve().parent/scene
    scene = scene.resolve()
    if cfg['samples'] < 1 or cfg['voxel_m'] <= 0 or cfg['joint_margin_rad'] < 0 or cfg['point_width_m'] <= 0:
        parser.error('Invalid samples, voxel size, joint margin or point width')
    out = (args.out or ROOT/'runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error('Use a fresh output directory to preserve previous results')
    out.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.Open(str(scene))
    if not stage or not stage.GetPrimAtPath(cfg['robot_root']):
        raise ValueError('Scene/robot root could not be opened')
    unit = UsdGeom.GetStageMetersPerUnit(stage)
    root_world = rigid(matrix(stage.GetPrimAtPath(cfg['robot_root'])), cfg['robot_root'])
    models, samples, world, voxels, selected = {}, {}, {}, {}, {}
    band = table_filter(stage, unit, cfg)
    report = dict(source_scene=str(scene), source_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        robot_root=cfg['robot_root'], samples_per_arm=cfg['samples'], seed=cfg['seed'],
        stage_meters_per_unit=unit, up_axis=str(UsdGeom.GetStageUpAxis(stage)),
        scope='Independent joint-limit FK samples of each flange. No IK, collision, grasp, path or dynamics tests.',
        arms={}, voxel_m=cfg['voxel_m'], height_filter=band, world_from_robot_root=metres(root_world, unit).tolist())
    for side in ['left', 'right']:
        spec = cfg.get('arms', {}).get(side, {})
        model = discover(stage, cfg['robot_root'], side, spec.get('base_prim'), spec.get('tip_prim'))
        models[side] = model
        lo, hi = model.lower+cfg['joint_margin_rad'], model.upper-cfg['joint_margin_rad']
        if np.any(lo >= hi):
            raise ValueError('Joint margin exceeds range')
        u = qmc.Sobol(7, scramble=True, seed=cfg['seed']).random_base2(int(np.ceil(np.log2(cfg['samples']))))[:cfg['samples']]
        q = lo + u*(hi-lo)
        t = model.fk(q)
        frames = dict(world=metres(t, unit), arm_base=metres(np.linalg.inv(model.base)@t, unit),
                      robot_root=metres(np.linalg.inv(root_world)@t, unit))
        samples[side], world[side] = q, frames['world']
        np.savez_compressed(out/(side+'_fk.npz'), sample_id=np.arange(len(q)), arm_q=q,
                            pose_world=frames['world'], pose_arm_base=frames['arm_base'], pose_robot_root=frames['robot_root'],
                            world_from_arm_base=metres(model.base, unit), world_from_robot_root=metres(root_world, unit),
                            joint_names=np.array(model.names))
        save_csv(out/(side+'_fk.csv'), q, frames)
        keep = np.flatnonzero(height_mask(frames['world'], band))
        selected[side] = keep
        filtered_frames = {key: value[keep] for key, value in frames.items()}
        save_csv(out/(side+'_fk_filtered.csv'), q[keep], filtered_frames, keep)
        np.savez_compressed(out/(side+'_fk_filtered.npz'), sample_id=keep, arm_q=q[keep],
            pose_world=frames['world'][keep], pose_arm_base=frames['arm_base'][keep], pose_robot_root=frames['robot_root'][keep],
            world_from_arm_base=metres(model.base, unit), world_from_robot_root=metres(root_world, unit), joint_names=np.array(model.names))
        v, first, counts = np.unique(np.floor(frames['world'][keep, :3, 3]/cfg['voxel_m']).astype(np.int64), axis=0, return_index=True, return_counts=True)
        ids = keep[first]
        voxels[side] = {tuple(x): (int(i), int(n)) for x, i, n in zip(v, ids, counts)}
        report['arms'][side] = dict(base_prim=model.base_path, tip_prim=model.tip_path, joint_names=model.names,
            lower_rad=model.lower.tolist(), upper_rad=model.upper.tolist(), sampled_lower_rad=lo.tolist(), sampled_upper_rad=hi.tolist(),
            world_from_arm_base=metres(model.base, unit).tolist(), last_body_from_tip=metres(model.tip_offset, unit).tolist(),
            world_bounds_m=dict(min=frames['world'][:, :3, 3].min(0).tolist(), max=frames['world'][:, :3, 3].max(0).tolist()),
            occupied_voxels=len(v), sampled_voxel_volume_m3=len(v)*cfg['voxel_m']**3,
            selected_count=len(keep), rejected_by_table_filter=len(q)-len(keep), preview_sample_id=int(keep[0]) if len(keep) else 0,
            authored_q_rad=model.authored_q.tolist(), authored_tip_matrix_error=model.authored_tip_error,
            authored_joint_matrix_error=model.authored_joint_error)
        print('FK_COMPLETE', side, len(q), 'SELECTED', len(keep), model.tip_path, flush=True)
    common = sorted(voxels['left'].keys() & voxels['right'].keys())
    centres = (np.asarray(common, dtype=int).reshape(-1, 3)+.5)*cfg['voxel_m']
    markers = np.asarray([(world['left'][voxels['left'][v][0], :3, 3]+world['right'][voxels['right'][v][0], :3, 3])/2 for v in common]).reshape(-1, 3)
    with (out/'shared_position_voxels.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['voxel_x', 'voxel_y', 'voxel_z', 'center_x_m', 'center_y_m', 'center_z_m',
                         'left_sample_id', 'right_sample_id', 'left_sample_count', 'right_sample_count', 'representative_distance_m',
                         'display_x_m', 'display_y_m', 'display_z_m'])
        for voxel, center, marker in zip(common, centres, markers):
            li, ln = voxels['left'][voxel]; ri, rn = voxels['right'][voxel]
            distance = np.linalg.norm(world['left'][li, :3, 3]-world['right'][ri, :3, 3])
            writer.writerow([*voxel, *center, li, ri, ln, rn, distance, *marker])
    report['shared_position_voxels'] = len(common)
    report['union_position_voxels'] = len(voxels['left'].keys() | voxels['right'].keys())
    report['shared_voxel_note'] = 'World cells occupied by selected samples from each arm separately. Display uses the mean of two representative positions to remain inside the height band/convex footprint. Neither cell centres nor displayed means are certified FK targets. No simultaneous feasibility guarantee.'
    preview(stage, out, models, samples, world, selected, markers, unit, cfg, band)
    html_view(out, world, selected, markers, models, unit, cfg['html_max_points_per_arm'], band)
    report['source_layers'] = [dict(path=l.realPath, sha256=hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest()) for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()]
    report['snapshot_sha256'] = hashlib.sha256((out/'input_scene.usdc').read_bytes()).hexdigest()
    (out/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    cfg['scene'] = str(scene)
    (out/'config_used.json').write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding='utf-8')
    (out.parent/'latest_success.json').write_text(json.dumps(dict(run_dir=str(out)), indent=2), encoding='utf-8')
    print('RUN_DIRECTORY', out, flush=True)
    print('SHARED_POSITION_VOXELS', len(common), flush=True)


if __name__ == '__main__':
    main()
