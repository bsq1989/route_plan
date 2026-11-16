"""MuJoCo visual assets for the Tracer chassis and Piper X arm."""

import hashlib
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import UsdGeom, UsdShade


STL_FACE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("tags", "<u2")])

# SolidWorks exports the stock Tracer parts in face order. These ranges refer to
# the pinned base_link.STL from AgileX, whose checksum is checked before use.
TRACER_SHELL_SHA256 = "3a243754343dd785d4169577224fbe581c1bd3fa24f46a7766a69bcf5ce03bf6"
TRACER_FINISH = {"specular": "0.22", "shininess": "0.2"}
TRACER_PAINT = {
    "deck": (57, 59, 62),
    "panel": (222, 225, 228),
    "rail": (149, 152, 156),
    "trim": (33, 35, 38),
    "stop": (179, 30, 35),
    "light": (234, 244, 255),
    "indicator": (125, 230, 34),
}


def add_textured_material(asset, name, path, *, specular, shininess):
    ET.SubElement(asset, "texture", name=name, type="2d", file=path.name)
    ET.SubElement(
        asset, "material", name=name, texture=name, rgba="1 1 1 1", specular=specular, shininess=shininess
    )


def write_textured_obj(path, points, faces, texcoords, uv_faces):
    vertices, vertex_indices = np.unique(points, axis=0, return_inverse=True)
    indices = np.stack((vertex_indices[faces] + 1, uv_faces + 1), axis=-1)
    with path.open("w", encoding="ascii") as output:
        np.savetxt(output, vertices, fmt="v %.9g %.9g %.9g")
        np.savetxt(output, texcoords, fmt="vt %.9g %.9g")
        np.savetxt(output, indices.reshape(-1, 6), fmt="f %d/%d %d/%d %d/%d")


def tracer_atlas(texture_path, triangles):
    image = Image.new("RGB", (512, 512), TRACER_PAINT["deck"])
    draw = ImageDraw.Draw(image)
    palette_uv = {}
    for index, (name, color) in enumerate(TRACER_PAINT.items()):
        x = 32 * index
        draw.rectangle((x, 0, x + 31, 31), fill=color)
        palette_uv[name] = ((x + 16) / 512, 1 - 16 / 512)
    uv = np.empty((len(triangles), 3, 2))
    uv[:] = palette_uv["deck"]
    for paint, ranges in (
        ("panel", ((1688, 3384), (21084, 22594), (25004, 27566))),
        ("rail", ((20100, 21084),)),
        ("trim", ((0, 1688), (3384, 8754), (8754, 10876), (10876, 12822),
                  (12902, 14590), (22594, 23426), (28386, 32278), (39474, 41420))),
    ):
        for start, end in ranges:
            uv[start:end] = palette_uv[paint]

    centers = triangles.mean(axis=1)
    for start, end in ((32402, 35876), (36000, 39474)):
        uv[start:end] = palette_uv["trim"]
        uv[start:end][np.abs(centers[start:end, 1]) > 0.295] = palette_uv["stop"]
    uv[19738:20100] = palette_uv["trim"]
    led = (centers[19738:20100, 0] > 0.307) & (np.abs(centers[19738:20100, 1]) < 0.091)
    uv[19738:20100][led] = palette_uv["light"]
    indicator = centers[23426:25004]
    ring = np.hypot(indicator[:, 1], indicator[:, 2] + 0.081) > 0.0065
    uv[23426:25004] = palette_uv["trim"]
    uv[23426:25004][ring & (indicator[:, 0] > 0.34)] = palette_uv["indicator"]

    # Project the labels onto the raised plates. The two sides face opposite directions.
    front = (20, 60, 492, 158)
    side = (20, 194, 492, 395)
    for box in (front, side):
        x0, y0, x1, y1 = box
        draw.rectangle((x0 - 4, y0 - 4, x1 + 4, y1 + 4), fill=TRACER_PAINT["trim"])
    draw.text((256, 109), "TRACER 2.0", font=ImageFont.load_default(size=62),
              fill=(214, 218, 221), anchor="mm")
    logo_font = ImageFont.load_default(size=96)
    draw.text((256, 271), "AGILE", font=logo_font, fill=(239, 241, 243), anchor="rm")
    draw.text((270, 271), "·X", font=logo_font, fill=(222, 56, 57), anchor="lm")
    draw.text((256, 343), "R o b o t i c s", font=ImageFont.load_default(size=38),
              fill=(218, 220, 223), anchor="mm")

    def label_uv(start, end, box, axis, reverse=False):
        points = triangles[start:end]
        x0, y0, x1, y1 = box
        x = points[..., axis]
        z = points[..., 2]
        u = (x - x.min()) / np.ptp(x)
        v = (z - z.min()) / np.ptp(z)
        if reverse:
            u = 1 - u
        uv[start:end, :, 0] = (x0 + u * (x1 - x0)) / 512
        uv[start:end, :, 1] = 1 - (y1 - v * (y1 - y0)) / 512

    label_uv(42636, 43754, front, 1)
    label_uv(42240, 42316, side, 0, reverse=True)
    label_uv(42316, 42392, side, 0)
    image.save(texture_path)
    coords, ids = np.unique(uv.reshape(-1, 2), axis=0, return_inverse=True)
    return coords, ids.reshape(-1, 3)


def add_tracer_shell_appearance(asset, geom, source, mesh_dir):
    if hashlib.sha256(source.read_bytes()).hexdigest() != TRACER_SHELL_SHA256:
        raise ValueError("The Tracer chassis texture requires the pinned AgileX base_link.STL.")
    name = "tracer2_shell_paint"
    mesh_path = mesh_dir / f"{name}.obj"
    texture_path = mesh_dir / f"{name}.png"
    if not mesh_path.is_file():
        triangles = np.fromfile(source, dtype=STL_FACE, offset=84)["vertices"]
        texcoords, uv_faces = tracer_atlas(texture_path, triangles)
        faces = np.arange(triangles.size // 3).reshape(-1, 3)
        write_textured_obj(mesh_path, triangles.reshape(-1, 3), faces, texcoords, uv_faces)
    add_textured_material(asset, name, texture_path, **TRACER_FINISH)
    ET.SubElement(asset, "mesh", name=name, file=mesh_path.name)
    geom.attrib.update(mesh=name, material=name, rgba="1 1 1 1")


def apply_tracer_deck_finish(asset, geom):
    name = "tracer2_deck_finish"
    rgba = " ".join(f"{channel / 255:.10g}" for channel in TRACER_PAINT["deck"]) + " 1"
    ET.SubElement(asset, "material", name=name, rgba=rgba, **TRACER_FINISH)
    geom.attrib.pop("rgba", None)
    geom.set("material", name)


def usd_diffuse_color(prim):
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if material:
        for shader in material.GetPrim().GetChildren():
            for input_name in ("inputs:diffuse_color_constant", "inputs:diffuseColor"):
                color = shader.GetAttribute(input_name)
                if color and color.HasAuthoredValueOpinion():
                    return tuple(color.Get())
    return (0.32, 0.35, 0.39)


def arm_face_palette(mesh, face_count):
    prim = mesh.GetPrim()
    colors = np.tile(usd_diffuse_color(prim), (face_count, 1))
    for subset in UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets():
        if subset.GetElementTypeAttr().Get() == UsdGeom.Tokens.face:
            indices = np.asarray(subset.GetIndicesAttr().Get(), dtype=np.intp)
            colors[indices] = usd_diffuse_color(subset.GetPrim())
    return np.unique(np.rint(np.clip(colors, 0, 1) * 255).astype(np.uint8), axis=0, return_inverse=True)


def write_arm_visual(asset, mesh_dir, name, mesh, points, faces):
    palette, face_colors = arm_face_palette(mesh, len(faces))
    if len(palette) == 1:
        rgba = " ".join(f"{value / 255:.6g}" for value in palette[0]) + " 1"
        return None, {"rgba": rgba}
    mesh_path = mesh_dir / f"{name}.obj"
    texture_path = mesh_dir / f"{name}.png"
    # Each solid-colour USD face uses the centre of its palette patch.
    patches = np.tile(np.repeat(palette, 8, axis=0)[None, ...], (8, 1, 1))
    Image.fromarray(patches).save(texture_path)
    texcoords = np.column_stack(((np.arange(len(palette)) + 0.5) / len(palette), np.full(len(palette), 0.5)))
    write_textured_obj(mesh_path, points, faces, texcoords, np.repeat(face_colors[:, None], 3, axis=1))
    add_textured_material(asset, name, texture_path, specular="0.24", shininess="0.25")
    return mesh_path.name, {"material": name}
