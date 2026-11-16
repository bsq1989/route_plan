"""Build the bare Tracer and mobile manipulator scenes from the copied assets."""

from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import struct
import tempfile
import xml.etree.ElementTree as ET

import fast_simplification
import mujoco
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from appearance import STL_FACE, add_tracer_shell_appearance, apply_tracer_deck_finish, write_arm_visual

PROJECT = Path(__file__).resolve().parent
SOURCE = PROJECT / "source"
DESCRIPTION = SOURCE / "mobile_manipulator2_description"
URDF = DESCRIPTION / "urdf/mobile_manipulator2_chassis_camera_hole_aligned.urdf"
TRACER_DESCRIPTION = SOURCE / "tracer2_description"
TRACER_URDF = TRACER_DESCRIPTION / "urdf/tracer2_description.urdf"
ARM_USD = SOURCE / "official_piper_x/piper_x_gripper_d435.usd"
MODEL = PROJECT / "model"

VISUAL = {"group": "2", "contype": "0", "conaffinity": "0", "density": "0"}
COLLISION = {"group": "3", "contype": "2", "conaffinity": "1", "rgba": "0.2 0.8 0.6 0.28"}


def numbers(values):
    return " ".join(f"{value:.10g}" for value in values)


def quaternion(rotation):
    return numbers((rotation.GetReal(), *rotation.GetImaginary()))


def joint_frame(position, rotation):
    frame = Gf.Transform()
    frame.SetTranslation(Gf.Vec3d(position))
    frame.SetRotation(Gf.Rotation(Gf.Quatd(rotation.GetReal(), Gf.Vec3d(rotation.GetImaginary()))))
    return frame


def save_mesh(path, points, faces):
    if len(faces) > 120_000:
        points, remap = np.unique(points, axis=0, return_inverse=True)
        points, faces = fast_simplification.simplify(
            points.astype(np.float64), remap[faces].astype(np.int32), target_count=120_000
        )
    triangles = np.zeros(len(faces), dtype=STL_FACE)
    triangles["vertices"] = points[faces]
    with path.open("wb") as output:
        output.write(bytes(80))
        output.write(struct.pack("<I", len(faces)))
        triangles.tofile(output)


def copy_stl(source, destination):
    with source.open("rb") as mesh:
        mesh.seek(80)
        (count,) = struct.unpack("<I", mesh.read(4))
        if count <= 120_000:
            shutil.copyfile(source, destination)
            return
        triangles = np.fromfile(mesh, dtype=STL_FACE, count=count)
    save_mesh(destination, triangles["vertices"].reshape(-1, 3), np.arange(count * 3).reshape(-1, 3))


def urdf_pose(element):
    origin = element.find("origin")
    if origin is None:
        return {}
    roll, pitch, yaw = map(float, origin.get("rpy", "0 0 0").split())
    rotation = (
        Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(roll))
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(pitch))
        * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))
    )
    return {"pos": origin.get("xyz", "0 0 0"), "quat": quaternion(rotation.GetQuat())}


def add_urdf_visuals(asset, body, link, package, mesh_dir, *, prefix=None, simplify=True):
    mesh_names = []
    for index, visual in enumerate(link.findall("visual")):
        mesh = visual.find("./geometry/mesh")
        relative = mesh.get("filename").removeprefix(f"package://{package.name}/")
        mesh_name = f"{prefix or body.get('name')}_visual_{index}"
        file = f"{mesh_name}.stl"
        source = package / relative
        if simplify:
            copy_stl(source, mesh_dir / file)
        else:
            shutil.copyfile(source, mesh_dir / file)
        ET.SubElement(asset, "mesh", name=mesh_name, file=file, scale=mesh.get("scale", "1 1 1"))
        color = visual.find("./material/color").get("rgba")
        ET.SubElement(
            body, "geom", name=mesh_name, type="mesh", mesh=mesh_name,
            rgba=color, **urdf_pose(visual), **VISUAL,
        )
        mesh_names.append(mesh_name)
    return mesh_names


def add_chassis_kit(asset, base, mesh_dir, robot):
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = {joint.get("name"): joint for joint in robot.findall("joint")}
    bodies = {"base_link": base}
    upper_kit_mass = 0.0
    # Keep the chassis and its mounted kit, with the display and camera mast omitted.
    for name in ("box_link", "lidar_link", "chassis_camera_link"):
        link = links[name]
        joint = next(joint for joint in joints.values() if joint.find("child").get("link") == name)
        parent = bodies[joint.find("parent").get("link")]
        bodies[name] = ET.SubElement(parent, "body", name=name, **urdf_pose(joint))
        upper_kit_mass += float(link.find("./inertial/mass").get("value"))
        add_urdf_visuals(asset, bodies[name], link, DESCRIPTION, mesh_dir)
    mount = joints["arm_joint"]
    arm_mount = ET.SubElement(
        bodies[mount.find("parent").get("link")], "body", name="piper_x_base", **urdf_pose(mount)
    )
    return arm_mount, upper_kit_mass


def add_official_chassis(asset, base, mesh_dir):
    robot = ET.parse(TRACER_URDF).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    shell_mesh, = add_urdf_visuals(
        asset, base, links["base_link"], TRACER_DESCRIPTION, mesh_dir,
        prefix="tracer2_base_link", simplify=False,
    )
    asset.find(f"mesh[@name='{shell_mesh}']").set("maxhullvert", "64")
    shell_visual = base.find(f"geom[@name='{shell_mesh}']")
    add_tracer_shell_appearance(asset, shell_visual, TRACER_DESCRIPTION / "meshes/base_link.STL", mesh_dir)
    collider = base.find("geom[@name='chassis_collision']")
    del collider.attrib["size"], collider.attrib["pos"]
    collider.attrib.update(type="mesh", mesh=shell_mesh)
    for joint in robot.findall("joint"):
        name = joint.find("child").get("link")
        pose = urdf_pose(joint)
        parent = base
        if joint.get("type") == "continuous":
            wheel_name = joint.get("name").replace("_joint", "_wheel")
            parent = base.find(f"body[@name='{wheel_name}']")
            # Express the URDF mesh frame in the active wheel's tread-centred frame.
            pose["pos"] = numbers(np.fromstring(pose["pos"], sep=" ") - np.fromstring(parent.get("pos"), sep=" "))
        body = ET.SubElement(parent, "body", name=f"tracer2_{name}", **pose)
        add_urdf_visuals(asset, body, links[name], TRACER_DESCRIPTION, mesh_dir, simplify=False)
        for geom in body.findall("geom"):
            geom.set("rgba", "0.12 0.12 0.13 1")


def add_drive(actuators, base, config, upper_kit_mass):
    radius, track = config["wheel_radius_m"], config["wheel_track_m"]
    wheel_mass, wheel_width = 1.0, 0.038
    mass = config["bare_base_mass_kg"] - 2 * wheel_mass + upper_kit_mass
    length, width, height = 0.702, 0.610, 0.169
    # Preserve the fitted inertia and primitive contact shapes of the Isaac prototype.
    inertia = (
        mass * (width**2 + height**2) / 12 + 0.15,
        mass * (length**2 + height**2) / 12 + 0.15,
        mass * (length**2 + width**2) / 12,
    )
    ET.SubElement(base, "inertial", pos="-0.015 0 -0.05", mass=str(mass), diaginertia=numbers(inertia))
    ET.SubElement(base, "geom", name="chassis_collision", type="box", size="0.34 0.28 0.05",
                  pos="0 0 -0.075", **COLLISION)
    if upper_kit_mass:
        ET.SubElement(base, "geom", name="upper_kit_collision", type="box", size="0.275 0.135 0.05",
                      pos="0.03 0 0.05", **COLLISION)
    for side, y in (("left", track / 2), ("right", -track / 2)):
        wheel = ET.SubElement(base, "body", name=f"{side}_wheel", pos=numbers((-0.018818, y, -0.0905)))
        joint_name = f"{side}_wheel_joint"
        ET.SubElement(wheel, "joint", name=joint_name, type="hinge", axis="0 1 0")
        ET.SubElement(
            wheel, "geom", name=f"{side}_tire", type="cylinder", size=numbers((radius, wheel_width / 2)),
            euler=f"{math.pi / 2} 0 0", mass=str(wheel_mass), friction="1 0.005 0.0001", **COLLISION,
        )
        ET.SubElement(actuators, "velocity", name=joint_name, joint=joint_name, kv="10",
                      forcerange="-8 8")
    for index, (x, y) in enumerate(((0.25, 0.20), (0.25, -0.20), (-0.25, 0.20), (-0.25, -0.20))):
        ET.SubElement(
            base, "geom", name=f"caster_{index}", type="sphere", size="0.039",
            pos=numbers((x, y, -0.114)), condim="1", priority="1", **COLLISION,
        )


def add_arm_meshes(asset, body, link, mesh_dir, cache):
    for section in ("visuals", "collisions"):
        group = link.GetChild(section)
        if not group:
            continue
        for index, prim in enumerate(Usd.PrimRange(group, Usd.TraverseInstanceProxies())):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            if not np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get()) == 3):
                raise ValueError(f"Expected triangles in {prim.GetPath()}")
            points = np.asarray(mesh.GetPointsAttr().Get())
            faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
            relative, _ = cache.ComputeRelativeTransform(prim, link)
            transform = np.asarray(relative)
            points = points @ transform[:3, :3] + transform[3, :3]
            left_handed = mesh.GetOrientationAttr().Get() == "leftHanded"
            if left_handed != (np.linalg.det(transform[:3, :3]) < 0):
                faces = faces[:, ::-1]
            name = f"piper_{link.GetName()}_{section}_{index}"
            if section == "visuals":
                file, surface = write_arm_visual(asset, mesh_dir, name, mesh, points, faces)
                appearance = {**VISUAL, **surface}
                attributes = {}
            else:
                file, appearance, attributes = None, COLLISION, {"maxhullvert": "64"}
            if file is None:
                file = f"{name}.stl"
                save_mesh(mesh_dir / file, points, faces)
            ET.SubElement(asset, "mesh", name=name, file=file, **attributes)
            ET.SubElement(body, "geom", name=name, type="mesh", mesh=name, **appearance)


def add_arm(asset, actuators, mount, mesh_dir):
    stage = Usd.Stage.Open(str(ARM_USD))
    root = stage.GetDefaultPrim()
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != "Z":
        raise ValueError("Expected a Z-up Piper X asset in metres")
    joints_by_parent = {}
    for prim in root.GetChild("joints").GetChildren():
        joint = UsdPhysics.Joint(prim)
        parent, = joint.GetBody0Rel().GetTargets()
        joints_by_parent.setdefault(parent, []).append(joint)
    cache = UsdGeom.XformCache()

    def add_link(link, body):
        mass = UsdPhysics.MassAPI(link)
        ET.SubElement(
            body, "inertial", mass=str(mass.GetMassAttr().Get()), pos=numbers(mass.GetCenterOfMassAttr().Get()),
            diaginertia=numbers(mass.GetDiagonalInertiaAttr().Get()), quat=quaternion(mass.GetPrincipalAxesAttr().Get()),
        )
        add_arm_meshes(asset, body, link, mesh_dir, cache)
        if link.GetName() == "camera_link":
            camera = UsdGeom.Camera(link.GetChild("Camera"))
            transform, _ = cache.ComputeRelativeTransform(camera.GetPrim(), link)
            fovy = 2 * math.atan(camera.GetVerticalApertureAttr().Get() / (2 * camera.GetFocalLengthAttr().Get()))
            ET.SubElement(body, "camera", name="wrist_d435", pos=numbers(transform.ExtractTranslation()),
                          quat=quaternion(transform.ExtractRotationQuat()), fovy=str(math.degrees(fovy)))
        for joint in joints_by_parent.get(link.GetPath(), []):
            prim = joint.GetPrim()
            child_path, = joint.GetBody1Rel().GetTargets()
            child = stage.GetPrimAtPath(child_path)
            anchor = joint_frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
            child_anchor = joint_frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
            transform = child_anchor.GetMatrix().GetInverse() * anchor.GetMatrix()
            child_body = ET.SubElement(
                body, "body", name=f"piper_{child.GetName()}",
                pos=numbers(transform.ExtractTranslation()), quat=quaternion(transform.ExtractRotationQuat()),
            )
            if not prim.IsA(UsdPhysics.FixedJoint):
                add_arm_joint(joint, child_anchor, child_body, actuators)
            add_link(child, child_body)

    add_link(root.GetChild("base_link"), mount)


def add_arm_joint(joint, child_anchor, body, actuators):
    prim = joint.GetPrim()
    hinge = prim.IsA(UsdPhysics.RevoluteJoint)
    axis = prim.GetAttribute("physics:axis").Get()
    axis_vector = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[axis]
    limits = tuple(prim.GetAttribute(f"physics:{bound}Limit").Get() for bound in ("lower", "upper"))
    drive_kind = "angular" if hinge else "linear"
    limit = prim.GetAttribute(f"drive:{drive_kind}:physics:maxForce").Get()
    name = str(prim.GetName())
    if hinge:
        limits = tuple(map(math.radians, limits))
    ET.SubElement(
        body, "joint", name=name, type="hinge" if hinge else "slide", range=numbers(limits),
        pos=numbers(child_anchor.GetTranslation()), axis=numbers(child_anchor.GetRotation().TransformDir(axis_vector)),
        armature="0.01" if hinge else "0.001",
    )
    ET.SubElement(
        actuators, "position", name=name, joint=name, ctrlrange=numbers(limits),
        forcerange=numbers((-limit, limit)), kp="160" if hinge else "800", kv="8" if hinge else "12",
    )


def build(destination):
    config = json.loads((SOURCE / "mobile_dynamics_config.json").read_text())
    project_robot = ET.parse(URDF).getroot()
    mesh_dir = destination / "meshes"
    mesh_dir.mkdir()
    root = ET.Element("mujoco", model="Official Tracer 2.0 + Piper X")
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes", texturedir="meshes", autolimits="true")
    ET.SubElement(root, "option", integrator="implicitfast", timestep="0.002")
    ET.SubElement(root, "statistic", center="0 0 0.35", extent="1.35")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", azimuth="135", elevation="-23", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient="0.28 0.28 0.28",
                  diffuse="0.53 0.53 0.53", specular="0.2 0.2 0.2")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  rgb1="0.24 0.29 0.34", rgb2="0.32 0.38 0.43", width="256", height="256")
    ET.SubElement(asset, "material", name="ground_grid", texture="grid", texrepeat="6 6", texuniform="true")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="1 -0.8 2.5", dir="-1 0.8 -2.5", diffuse="0.65 0.65 0.65")
    ET.SubElement(world, "geom", name="floor", type="plane", size="6 6 0.1", material="ground_grid",
                  contype="1", conaffinity="2")
    base = ET.SubElement(world, "body", name="base_link", pos="0 0 0.155")
    ET.SubElement(base, "freejoint", name="base_free")
    actuators = ET.SubElement(root, "actuator")

    bare_root = deepcopy(root)
    bare_root.set("model", "Official Tracer 2.0")
    bare_root.find("statistic").attrib.update(center="0 0 0.08", extent="0.9")
    bare_base = bare_root.find("./worldbody/body[@name='base_link']")
    add_drive(bare_root.find("actuator"), bare_base, config, 0)
    add_official_chassis(bare_root.find("asset"), bare_base, mesh_dir)

    mount, upper_kit_mass = add_chassis_kit(asset, base, mesh_dir, project_robot)
    add_drive(actuators, base, config, upper_kit_mass)
    add_arm(asset, actuators, mount, mesh_dir)

    project_root = deepcopy(root)
    project_root.set("model", "Project Tracer chassis + Piper X")
    add_urdf_visuals(
        project_root.find("asset"), project_root.find("./worldbody/body[@name='base_link']"),
        project_robot.find("link[@name='base_link']"), DESCRIPTION, mesh_dir,
    )
    add_official_chassis(asset, base, mesh_dir)
    apply_tracer_deck_finish(asset, base.find("body[@name='box_link']/geom"))

    models = {}
    scenes = (("scene_tracer.xml", bare_root), ("scene.xml", root), ("scene_project.xml", project_root))
    for filename, tree in scenes:
        scene = destination / filename
        ET.indent(tree)
        ET.ElementTree(tree).write(scene, encoding="utf-8", xml_declaration=True)
        models[filename] = mujoco.MjModel.from_xml_path(str(scene))
    return models


def main():
    if not all(path.is_file() for path in (URDF, TRACER_URDF, ARM_USD)):
        raise SystemExit("Missing source assets. Run mujoco-dev/fetch_assets.sh first.")
    with tempfile.TemporaryDirectory(dir=PROJECT) as workspace:
        generated = Path(workspace)
        models = build(generated)
        MODEL.mkdir(exist_ok=True)
        shutil.copytree(generated / "meshes", MODEL / "meshes", dirs_exist_ok=True)
        for filename, model in models.items():
            (generated / filename).replace(MODEL / filename)
            print(f"{MODEL / filename}: {model.nbody} bodies, {model.njnt} joints, {model.nu} actuators")


if __name__ == "__main__":
    main()
