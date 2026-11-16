"""Exercise the generated mobile manipulator without a display."""

import json
import math
from pathlib import Path

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parent
SCENE = PROJECT / "model/scene.xml"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
GRIPPER_JOINTS = ("gripper_joint1", "gripper_joint2")
DRIVE_JOINTS = ("left_wheel_joint", "right_wheel_joint")


def set_base_velocity(model, data, forward, yaw_rate):
    """Set wheel targets from base-frame forward m/s and angular rad/s."""
    radius = model.geom("left_tire").size[0]
    track = model.body("left_wheel").pos[1] - model.body("right_wheel").pos[1]
    left, right = DRIVE_JOINTS
    data.actuator(left).ctrl[0] = (forward - yaw_rate * track / 2) / radius
    data.actuator(right).ctrl[0] = (forward + yaw_rate * track / 2) / radius


def step(model, data, seconds):
    count = round(seconds / model.opt.timestep)
    mujoco.mj_step(model, data, nstep=count)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise RuntimeError("Non-finite simulation state")
    warnings = [warning.number for warning in data.warning]
    if any(warnings):
        raise RuntimeError(f"MuJoCo warnings: {warnings}")
    mujoco.mj_forward(model, data)


def base_pose(data):
    x, y, _ = data.body("base_link").xpos
    matrix = data.body("base_link").xmat.reshape(3, 3)
    return np.array((x, y, math.atan2(matrix[1, 0], matrix[0, 0])))


def check(model):
    data = mujoco.MjData(model)
    step(model, data, 2.0)
    start = base_pose(data)
    base_z = data.body("base_link").xpos[2]
    if np.linalg.norm(start[:2]) > 0.02 or not 0.14 < base_z < 0.17:
        raise RuntimeError(f"Unstable idle pose: {start}, z={base_z:.4f}")

    set_base_velocity(model, data, 0.25, 0)
    step(model, data, 2.0)
    straight = base_pose(data) - start
    if straight[0] < 0.30 or abs(straight[1]) > 0.04 or abs(straight[2]) > 0.1:
        raise RuntimeError(f"Unexpected straight-line motion: {straight}")

    set_base_velocity(model, data, 0, 0)
    step(model, data, 1.0)
    stopped = base_pose(data)
    step(model, data, 1.0)
    stopped_drift = np.linalg.norm(base_pose(data)[:2] - stopped[:2])
    if stopped_drift > 0.02:
        raise RuntimeError(f"Unexpected drift after stop: {stopped_drift:.4f} m")

    start_turn = base_pose(data)
    set_base_velocity(model, data, 0, 0.65)
    step(model, data, 1.5)
    turn = base_pose(data) - start_turn
    if turn[2] < 0.55 or np.linalg.norm(turn[:2]) > 0.08:
        raise RuntimeError(f"Unexpected point-turn motion: {turn}")

    arm_start = np.array([data.joint(name).qpos[0] for name in ARM_JOINTS])
    arm_target = np.array([0.25, 0.25, -0.35, 0.20, -0.20, 0.20])
    jaw_target = np.array([0.02, -0.02])
    for name, target in zip(ARM_JOINTS + GRIPPER_JOINTS, np.concatenate((arm_target, jaw_target)), strict=True):
        data.actuator(name).ctrl[0] = target
    set_base_velocity(model, data, 0.15, 0)
    coordinated_start = base_pose(data)
    step(model, data, 2.0)
    coordinated_motion = np.linalg.norm(base_pose(data)[:2] - coordinated_start[:2])
    arm_final = np.array([data.joint(name).qpos[0] for name in ARM_JOINTS])
    jaw_final = np.array([data.joint(name).qpos[0] for name in GRIPPER_JOINTS])
    if coordinated_motion < 0.15 or np.any(np.abs(arm_final - arm_start) < 0.10):
        raise RuntimeError(f"Unexpected coordinated motion: {coordinated_motion:.3f} m, {arm_final - arm_start}")
    arm_error = np.max(np.abs(arm_final - arm_target))
    jaw_error = np.max(np.abs(jaw_final - jaw_target))
    if arm_error > 0.12 or jaw_error > 0.003:
        raise RuntimeError(f"Unexpected tracking error: arm {arm_error:.4f} rad, jaws {jaw_error:.4f} m")

    return {
        "mujoco": mujoco.__version__,
        "bodies": model.nbody,
        "joints": model.njnt,
        "actuators": model.nu,
        "idle_base_height_m": round(base_z, 4),
        "straight_dx_m": round(straight[0], 4),
        "straight_lateral_error_m": round(abs(straight[1]), 4),
        "stopped_drift_m": round(stopped_drift, 4),
        "point_turn_rad": round(turn[2], 4),
        "point_turn_translation_m": round(np.linalg.norm(turn[:2]), 4),
        "combined_drive_travel_m": round(coordinated_motion, 4),
        "max_arm_error_rad": round(arm_error, 4),
        "max_jaw_error_m": round(jaw_error, 4),
    }


if __name__ == "__main__":
    if not SCENE.is_file():
        raise SystemExit("Missing scene. Run mujoco-dev/build_model.py first.")
    report = check(mujoco.MjModel.from_xml_path(str(SCENE)))
    output = PROJECT / "output"
    output.mkdir(exist_ok=True)
    result = json.dumps(report, indent=2)
    (output / "check.json").write_text(result + "\n")
    print(result)
