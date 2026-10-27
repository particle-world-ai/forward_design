"""Cross-check FK against spatial screw composition and authored USD transforms."""
import json
import argparse
from pathlib import Path
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom
from dual_fk import ArmFK, matrix

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    out = parser.parse_args().results_dir.resolve()
    report = json.loads((out/'summary.json').read_text(encoding='utf-8'))
    stage = Usd.Stage.Open(str(out/'input_scene.usdc'))
    preview = Usd.Stage.Open(str(out/'workspace_preview.usda'))
    unit = report['stage_meters_per_unit']
    validation = {}
    for side, spec in report['arms'].items():
        model = ArmFK(stage, report['robot_root'], spec['base_prim'], spec['tip_prim'])
        data = np.load(out/(side+'_fk.npz'))
        q = data['arm_q']
        assert q.shape == (report['samples_per_arm'], 7)
        assert np.all(q >= model.lower) and np.all(q <= model.upper)
        worst = 0.
        for i in np.linspace(0, len(q)-1, 64, dtype=int):
            # Space-frame product of exponentials about axes measured in the authored USD.
            correction = np.eye(4)
            for a, b, index, axis, parent, child in model.chain:
                if index is None:
                    continue
                joint_world = matrix(stage.GetPrimAtPath(parent)) @ a
                direction, pivot = joint_world[:3, :3] @ axis, joint_world[:3, 3]
                rot = Rotation.from_rotvec(direction*(q[i, index]-model.authored_q[index])).as_matrix()
                screw = np.eye(4)
                screw[:3, :3], screw[:3, 3] = rot, pivot-rot@pivot
                correction = correction @ screw
            pose = correction @ matrix(stage.GetPrimAtPath(model.tip_path))
            pose[:3, 3] *= unit
            worst = max(worst, float(abs(pose-data['pose_world'][i]).max()))
        assert worst < 2e-5, worst
        base_error = np.max(abs(data['world_from_arm_base'] @ data['pose_arm_base']-data['pose_world']))
        root_error = np.max(abs(data['world_from_robot_root'] @ data['pose_robot_root']-data['pose_world']))
        assert max(base_error, root_error) < 1e-10
        rendered_tip = matrix(preview.GetPrimAtPath(model.tip_path))
        rendered_tip[:3, 3] *= unit
        preview_error = np.max(abs(rendered_tip-data['pose_world'][spec.get('preview_sample_id', 0)]))
        assert preview_error < 1e-6, preview_error
        cloud = np.array(UsdGeom.Points(preview.GetPrimAtPath('/DualWorkspace/'+side.capitalize()+'FK')).GetPointsAttr().Get()).reshape(-1, 3)*unit
        selected_path = out/(side+'_fk_filtered.npz')
        selected_ids = np.load(selected_path)['sample_id'] if selected_path.exists() else np.arange(len(q))
        assert cloud.shape == (len(selected_ids), 3)
        if len(cloud):
            assert np.max(abs(cloud-data['pose_world'][selected_ids, :3, 3])) < 1e-6
        band = report.get('height_filter')
        if band:
            # Independent membership check in the rectangular table's world XY footprint.
            table = stage.GetPrimAtPath(band['table_prim'])
            raw = np.array(UsdGeom.Mesh(table).GetPointsAttr().Get())
            tw = matrix(table)
            vertices = (raw @ tw[:3, :3].T+tw[:3, 3])*unit
            assert abs(vertices[:, 2].max()-band['table_top_world_z_m']) < 1e-12
            p = data['pose_world'][:, :3, 3]
            actual_height = p[:, 2]-vertices[:, 2].max()
            expected = (actual_height >= band['min_height_m']-1e-10) & (actual_height <= band['max_height_m']+1e-10)
            if band['table_xy_only']:
                # Ray crossing against the convex footprint, without reusing the halfspace filter.
                polygon = np.array(band['xy_polygon_m'])
                inside = np.zeros(len(p), dtype=bool)
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    if abs(b[1]-a[1]) > 1e-15:
                        inside ^= ((a[1] > p[:, 1]) != (b[1] > p[:, 1])) & (p[:, 0] < (b[0]-a[0])*(p[:, 1]-a[1])/(b[1]-a[1])+a[0])
                expected &= inside
            assert np.array_equal(selected_ids, np.flatnonzero(expected))
            filtered = np.load(selected_path)
            assert np.array_equal(filtered['arm_q'], q[selected_ids])
            assert np.array_equal(filtered['pose_world'], data['pose_world'][selected_ids])
            filtered_csv = np.atleast_1d(np.genfromtxt(out/(side+'_fk_filtered.csv'), delimiter=',', names=True))
            assert np.array_equal(filtered_csv['sample_id'].astype(int), selected_ids)
            assert UsdGeom.Imageable(preview.GetPrimAtPath('/DualWorkspace/'+side.capitalize()+'FKAll')).ComputeVisibility() == UsdGeom.Tokens.invisible
        csv_data = np.atleast_1d(np.genfromtxt(out/(side+'_fk.csv'), delimiter=',', names=True, encoding='utf-8'))
        assert len(csv_data) == len(q) and np.array_equal(csv_data['sample_id'], np.arange(len(q)))
        assert np.max(abs(csv_data['world_x_m']-data['pose_world'][:, 0, 3])) < 1e-10
        validation[side] = dict(samples=len(q), selected_count=len(selected_ids), independent_screw_fk_max_matrix_error=worst,
            preview_tip_max_matrix_error=float(preview_error), base_roundtrip_error=float(base_error), root_roundtrip_error=float(root_error))
    assert hashlib.sha256(Path(report['source_scene']).read_bytes()).hexdigest() == report['source_sha256']
    for layer in report['source_layers']:
        assert hashlib.sha256(Path(layer['path']).read_bytes()).hexdigest() == layer['sha256']
    # Relocate the robot only in a temporary session layer: all world samples must follow it.
    stage.SetEditTarget(stage.GetSessionLayer())
    robot = stage.GetPrimAtPath(report['robot_root'])
    original_root = matrix(robot)
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler('z', .43).as_matrix()
    delta[:3, 3] = [.21/unit, -.16/unit, .08/unit]
    new_local = np.linalg.inv(matrix(robot.GetParent())) @ delta @ original_root
    xf = UsdGeom.Xformable(robot)
    xf.ClearXformOpOrder()
    xf.AddTransformOp(opSuffix='validationOnly').Set(Gf.Matrix4d(new_local.T.tolist()))
    moved_errors = {}
    for side, spec in report['arms'].items():
        moved = ArmFK(stage, report['robot_root'], spec['base_prim'], spec['tip_prim'])
        data = np.load(out/(side+'_fk.npz'))
        actual = moved.fk(data['arm_q'][:32])
        original = data['pose_world'][:32].copy()
        original[:, :3, 3] /= unit
        error = float(np.max(abs(actual-delta@original)))
        assert error < 1e-6
        moved_errors[side] = error
    (out/'validation.json').write_text(json.dumps(dict(passed=True, arms=validation, original_layers_unchanged=True,
        moved_robot_world_transform_errors=moved_errors), indent=2), encoding='utf-8')
    print(json.dumps(validation, indent=2))
    print('FK_VALIDATION_PASSED')


if __name__ == '__main__':
    main()
