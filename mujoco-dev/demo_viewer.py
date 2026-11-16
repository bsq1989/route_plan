"""Play the Tracer 2.0 and Piper X demo once and hold the final pose."""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from check_model import ARM_JOINTS, GRIPPER_JOINTS, SCENE, base_pose, set_base_velocity


JOINT_NAMES = ARM_JOINTS + GRIPPER_JOINTS
HOME_POSE = (0, 0, 0, 0, 0, 0, 0, 0)
OPEN_POSE = (0.32, 0.38, -0.47, 0.28, -0.30, 0.35, 0.043, -0.043)
CLOSED_POSE = (0.1, 0.19, -0.25, 0.13, -0.10, 0.12, 0.005, -0.005)
PHASES = (
    ("Ready", 3.0, 0, 0, HOME_POSE),
    ("Drive straight", 3.5, 0.19, 0, HOME_POSE),
    ("Stop", 1.0, 0, 0, HOME_POSE),
    ("Turn in place", 2.5, 0, 0.44, HOME_POSE),
    ("Drive + move arm + open gripper", 3.7, 0.15, 0, OPEN_POSE),
    ("Drive + move arm + close gripper", 3.7, 0.12, 0, CLOSED_POSE),
    ("Stop and hold", 1.0, 0, 0, CLOSED_POSE),
)


def run(model, data, viewer=None):
    if viewer is not None:
        with viewer.lock():
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -24
            viewer.cam.distance = 2.45
            viewer.cam.lookat[:] = [0.38, 0.26, 0.33]
        viewer.sync()
        print("Viewer ready", flush=True)

    steps_per_frame = 8
    frame_seconds = steps_per_frame * model.opt.timestep
    previous = np.array(HOME_POSE)
    for label, seconds, forward, yaw_rate, end_pose in PHASES:
        start = base_pose(data)
        if viewer is not None:
            viewer.set_texts((None, None, "Tracer 2.0 + Piper X", label))
        frame_count = round(seconds / frame_seconds)
        deadline = time.monotonic()
        set_base_velocity(model, data, forward, yaw_rate)
        target = np.array(end_pose)
        for frame in range(frame_count):
            if viewer is not None and not viewer.is_running():
                return False
            fraction = (frame + 1) / frame_count
            smooth = fraction * fraction * (3 - 2 * fraction)
            positions = previous + smooth * (target - previous)
            for name, position in zip(JOINT_NAMES, positions, strict=True):
                data.actuator(name).ctrl[0] = position
            mujoco.mj_step(model, data, nstep=steps_per_frame)
            if viewer is not None:
                viewer.sync()
                deadline += frame_seconds
                time.sleep(max(0, deadline - time.monotonic()))
        previous = target
        if not np.isfinite(data.qpos).all() or any(warning.number for warning in data.warning):
            raise RuntimeError(f"Invalid simulation state during {label}")
        change = base_pose(data) - start
        travel = np.linalg.norm(change[:2])
        left, right = (data.joint(name).qpos[0] for name in GRIPPER_JOINTS)
        print(f"{label}: travel={travel:.3f} m, yaw={change[2]:.3f} rad, jaws={left - right:.3f} m", flush=True)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Run the same sequence without a window.")
    args = parser.parse_args()
    if not SCENE.is_file():
        raise SystemExit("Missing scene. Run mujoco-dev/build_model.py first.")

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    if args.check:
        run(model, data)
        return

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        if run(model, data, viewer):
            viewer.set_texts((None, None, "Tracer 2.0 + Piper X", "Demo complete"))
            print("Demo complete. Close the viewer to exit.", flush=True)
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)


if __name__ == "__main__":
    main()
