#!/usr/bin/env python3

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


WORLD_NAME = "parking_lot_scaled_vehicles.world"
_SCRIPT_DIR = Path(__file__).resolve().parent
_LIMO_GAZEBOSIM_DIR = _SCRIPT_DIR.parent
MANAGED_INCLUDE_NAMES = (
    "scaled_prius_hybrid_instance",
    "scaled_golf_cart_instance",
    "scaled_delivery_truck_instance",
    "scaled_passenger_car_instance",
    "scaled_suv_instance",
)
MANAGED_INCLUDE_URIS = (
    "model://scaled_prius_hybrid",
    "model://scaled_golf_cart",
    "model://scaled_delivery_truck",
    "model://scaled_passenger_car",
    "model://scaled_suv",
)
MODEL_SPECS = (
    {
        "requested_name": "prius_hybrid",
        "official_source": "prius_hybrid",
        "source_root": "official",
        "scale_factor": 0.14,
        "note": "Exact official Gazebo model, scaled to 1.4x the previous 1/10 size.",
    },
    {
        "requested_name": "passenger_car",
        "official_source": "hatchback_red",
        "source_root": "official",
        "scale_factor": 0.14,
        "note": "Official Gazebo hatchback used as the additional scaled passenger car, enlarged to 1.4x the previous 1/10 size.",
    },
    {
        "requested_name": "suv",
        "official_source": "suv",
        "source_root": "local",
        "scale_factor": 0.1,
        "note": "Local Gazebo SUV model scaled for the parking scenario.",
    },
)
WORLD_LAYOUT = (
    ("scaled_prius_hybrid_instance", "scaled_prius_hybrid", (-1.5, 0.0, 0.02, 0.0, 0.0, 0.0)),
    ("scaled_passenger_car_instance", "scaled_passenger_car", (0.0, 0.0, 0.02, 0.0, 0.0, 0.0)),
    ("scaled_suv_instance", "scaled_suv", (1.5, 0.0, 0.02, 0.0, 0.0, 0.0)),
)
MANAGED_MODEL_DIRS = (
    "golf_cart",
    "delivery_truck",
    "scaled_golf_cart",
    "scaled_delivery_truck",
    "prius_hybrid",
    "scaled_prius_hybrid",
    "passenger_car",
    "scaled_passenger_car",
    "suv",
    "scaled_suv",
)


def strip_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1]


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            indent_xml(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def scaled_numbers(text: str, scale: float, pose: bool = False) -> str:
    values = [float(item) for item in text.split()]
    if pose:
        for index in range(min(3, len(values))):
            values[index] *= scale
    else:
        values = [value * scale for value in values]
    return " ".join(format_number(value) for value in values)


def format_number(value: float) -> str:
    text = f"{value:.8f}".rstrip("0").rstrip(".")
    return text if text else "0"


def ensure_model_static(model_elem: ET.Element) -> None:
    static_elem = None
    for child in model_elem:
        if strip_namespace(child.tag) == "static":
            static_elem = child
            break
    if static_elem is None:
        static_elem = ET.Element("static")
        static_elem.text = "true"
        insert_index = 0
        for index, child in enumerate(list(model_elem)):
            if strip_namespace(child.tag) == "pose":
                insert_index = index + 1
                break
        model_elem.insert(insert_index, static_elem)
    else:
        static_elem.text = "true"


def ensure_mesh_scale(mesh_elem: ET.Element, scale: float) -> None:
    scale_elem = None
    for child in mesh_elem:
        if strip_namespace(child.tag) == "scale":
            scale_elem = child
            break
    if scale_elem is None:
        scale_elem = ET.Element("scale")
        scale_elem.text = f"{scale} {scale} {scale}"
        mesh_elem.append(scale_elem)
    else:
        scale_elem.text = scaled_numbers(scale_elem.text or "1 1 1", scale, pose=False)


def patch_model_config(config_path: Path, model_name: str, note: str) -> None:
    tree = ET.parse(config_path)
    root = tree.getroot()
    name_elem = root.find("name")
    if name_elem is None:
        name_elem = ET.SubElement(root, "name")
    name_elem.text = model_name

    description_elem = root.find("description")
    description_text = (
        f"{model_name}. Auto-generated 1/10 static vehicle model. Source note: {note}"
    )
    if description_elem is None:
        description_elem = ET.SubElement(root, "description")
    description_elem.text = description_text

    indent_xml(root)
    tree.write(config_path, encoding="utf-8", xml_declaration=True)


def scale_sdf_model(
    sdf_path: Path,
    model_name: str,
    model_uri_name: str,
    scale: float,
) -> None:
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_elem = next((elem for elem in root.iter() if strip_namespace(elem.tag) == "model"), None)
    if model_elem is None:
        raise RuntimeError(f"No <model> found in {sdf_path}")

    model_elem.set("name", model_name)
    ensure_model_static(model_elem)

    for elem in root.iter():
        tag = strip_namespace(elem.tag)
        if tag == "pose" and elem.text:
            elem.text = scaled_numbers(elem.text, scale, pose=True)
        elif tag == "size" and elem.text:
            elem.text = scaled_numbers(elem.text, scale, pose=False)
        elif tag in {"radius", "length"} and elem.text:
            elem.text = format_number(float(elem.text) * scale)
        elif tag == "mesh":
            ensure_mesh_scale(elem, scale)
        elif tag == "uri" and elem.text and elem.text.strip().startswith("model://"):
            uri_text = elem.text.strip()
            uri_parts = uri_text.split("/", 3)
            if len(uri_parts) >= 3:
                uri_parts[2] = model_uri_name
                elem.text = "/".join(uri_parts)

    indent_xml(root)
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)


def alias_unscaled_model(
    source_dir: Path,
    dest_dir: Path,
    model_name: str,
    source_name: str,
    note: str,
) -> None:
    if dest_dir.exists():
        shutil.rmtree(dest_dir)
    shutil.copytree(source_dir, dest_dir)

    config_path = dest_dir / "model.config"
    sdf_path = dest_dir / "model.sdf"
    if config_path.exists():
        patch_model_config(config_path, model_name, note)
    if sdf_path.exists():
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model_elem = next(
            (elem for elem in root.iter() if strip_namespace(elem.tag) == "model"),
            None,
        )
        if model_elem is not None:
            model_elem.set("name", model_name)
        for elem in root.iter():
            if strip_namespace(elem.tag) == "uri" and elem.text:
                elem.text = elem.text.replace(f"model://{source_name}", f"model://{model_name}")
        indent_xml(root)
        tree.write(sdf_path, encoding="utf-8", xml_declaration=True)


def create_scaled_model(
    original_dir: Path,
    scaled_dir: Path,
    scaled_name: str,
    note: str,
    scale_factor: float,
) -> None:
    if scaled_dir.exists():
        shutil.rmtree(scaled_dir)
    shutil.copytree(original_dir, scaled_dir)

    config_path = scaled_dir / "model.config"
    sdf_path = scaled_dir / "model.sdf"
    if config_path.exists():
        patch_model_config(config_path, scaled_name, note)
    if sdf_path.exists():
        scale_sdf_model(sdf_path, scaled_name, scaled_name, scale_factor)


def update_world(world_path: Path) -> None:
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_elem = next((elem for elem in root.iter() if strip_namespace(elem.tag) == "world"), None)
    if world_elem is None:
        raise RuntimeError(f"No <world> found in {world_path}")

    for child in list(world_elem):
        tag = strip_namespace(child.tag)
        if tag == "model" and child.get("name") == "parked_suv":
            world_elem.remove(child)
        elif tag == "include":
            include_name = None
            include_uri = None
            for grandchild in child:
                if strip_namespace(grandchild.tag) == "name":
                    include_name = (grandchild.text or "").strip()
                elif strip_namespace(grandchild.tag) == "uri":
                    include_uri = (grandchild.text or "").strip()
            if include_name in MANAGED_INCLUDE_NAMES or include_uri in MANAGED_INCLUDE_URIS:
                world_elem.remove(child)

    for include_name, model_name, pose in WORLD_LAYOUT:
        include_elem = ET.Element("include")
        uri_elem = ET.SubElement(include_elem, "uri")
        uri_elem.text = f"model://{model_name}"
        name_elem = ET.SubElement(include_elem, "name")
        name_elem.text = include_name
        pose_elem = ET.SubElement(include_elem, "pose")
        pose_elem.text = " ".join(format_number(value) for value in pose)
        world_elem.append(include_elem)

    indent_xml(root)
    tree.write(world_path, encoding="utf-8", xml_declaration=True)


def cleanup_managed_model_dirs(models_root: Path) -> None:
    for directory_name in MANAGED_MODEL_DIRS:
        target = models_root / directory_name
        if target.exists():
            shutil.rmtree(target)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prepare scaled local Gazebo vehicle models and place them in the scaled parking world."
    )
    parser.add_argument(
        "--official-source-root",
        type=Path,
        default=Path("/tmp/gazebo_models_osrf"),
        help="Path to the cloned official Gazebo models repository.",
    )
    parser.add_argument(
        "--local-source-root",
        type=Path,
        default=Path.home() / ".gazebo" / "models",
        help="Path to the local Gazebo model cache.",
    )
    parser.add_argument(
        "--models-root",
        type=Path,
        default=_LIMO_GAZEBOSIM_DIR / "models",
        help="Target local models directory.",
    )
    parser.add_argument(
        "--world",
        type=Path,
        default=_LIMO_GAZEBOSIM_DIR / "worlds" / WORLD_NAME,
        help="Target scaled parking world file.",
    )
    args = parser.parse_args()

    args.models_root.mkdir(parents=True, exist_ok=True)
    cleanup_managed_model_dirs(args.models_root)

    for spec in MODEL_SPECS:
        source_root = (
            args.official_source_root
            if spec["source_root"] == "official"
            else args.local_source_root
        )
        source_dir = source_root / spec["official_source"]
        if not source_dir.exists():
            raise FileNotFoundError(
                f"Missing source model '{spec['official_source']}' in {source_root}"
            )

        original_dir = args.models_root / spec["requested_name"]
        scaled_dir = args.models_root / f"scaled_{spec['requested_name']}"

        alias_unscaled_model(
            source_dir=source_dir,
            dest_dir=original_dir,
            model_name=spec["requested_name"],
            source_name=spec["official_source"],
            note=spec["note"],
        )
        create_scaled_model(
            original_dir=original_dir,
            scaled_dir=scaled_dir,
            scaled_name=f"scaled_{spec['requested_name']}",
            note=spec["note"],
            scale_factor=spec["scale_factor"],
        )

    update_world(args.world)
    print(f"Prepared local models in: {args.models_root}")
    print(f"Updated world: {args.world}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
