#!/usr/bin/env python3

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


TARGET_SCALE = "0.1 0.1 0.1"
VEHICLE_KEYWORDS = (
    "car",
    "vehicle",
    "suv",
    "hatchback",
    "sedan",
    "truck",
    "van",
    "pickup",
    "bus",
    "wagon",
    "coupe",
)
LIMO_KEYWORDS = ("limo",)


def strip_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1]


def iter_child_elements(parent: ET.Element, name: str):
    for child in parent:
        if strip_namespace(child.tag) == name:
            yield child


def is_limo_name(value: str) -> bool:
    lowered = value.lower()
    return any(keyword in lowered for keyword in LIMO_KEYWORDS)


def is_vehicle_name(value: str) -> bool:
    lowered = value.lower()
    return any(keyword in lowered for keyword in VEHICLE_KEYWORDS)


def should_scale_model(model_elem: ET.Element) -> bool:
    name = model_elem.get("name", "")
    if is_limo_name(name):
        return False
    return is_vehicle_name(name)


def should_scale_include(include_elem: ET.Element) -> bool:
    names = []
    for uri in iter_child_elements(include_elem, "uri"):
        if uri.text:
            names.append(uri.text.strip())
    for name_elem in iter_child_elements(include_elem, "name"):
        if name_elem.text:
            names.append(name_elem.text.strip())

    combined = " ".join(names)
    if not combined or is_limo_name(combined):
        return False
    return is_vehicle_name(combined)


def upsert_scale(parent: ET.Element) -> bool:
    for child in parent:
        if strip_namespace(child.tag) == "scale":
            if (child.text or "").strip() != TARGET_SCALE:
                child.text = TARGET_SCALE
                return True
            return False

    scale_elem = ET.Element("scale")
    scale_elem.text = TARGET_SCALE
    insert_index = 0
    for index, child in enumerate(list(parent)):
        if strip_namespace(child.tag) == "geometry":
            insert_index = index
            break
    parent.insert(insert_index, scale_elem)
    return True


def upsert_include_scale(include_elem: ET.Element) -> bool:
    for child in include_elem:
        if strip_namespace(child.tag) == "scale":
            if (child.text or "").strip() != TARGET_SCALE:
                child.text = TARGET_SCALE
                return True
            return False

    scale_elem = ET.Element("scale")
    scale_elem.text = TARGET_SCALE

    insert_index = len(include_elem)
    for index, child in enumerate(list(include_elem)):
        if strip_namespace(child.tag) in {"pose", "name", "uri"}:
            insert_index = index + 1
    include_elem.insert(insert_index, scale_elem)
    return True


def update_visuals_and_collisions(container: ET.Element) -> int:
    updated = 0
    for element in container.iter():
        tag = strip_namespace(element.tag)
        if tag in {"visual", "collision"}:
            if upsert_scale(element):
                updated += 1
    return updated


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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scale non-LIMO vehicle models in a Gazebo world file."
    )
    parser.add_argument("input_world", type=Path)
    parser.add_argument("output_world", type=Path)
    args = parser.parse_args()

    tree = ET.parse(args.input_world)
    root = tree.getroot()
    root_copy = copy.deepcopy(root)

    updated_count = 0
    scaled_targets = []

    for model in root_copy.iter():
        if strip_namespace(model.tag) != "model":
            continue
        if should_scale_model(model):
            updated_count += update_visuals_and_collisions(model)
            scaled_targets.append(f"model:{model.get('name', 'unknown')}")

    for include in root_copy.iter():
        if strip_namespace(include.tag) != "include":
            continue
        if should_scale_include(include):
            if upsert_include_scale(include):
                updated_count += 1
            scaled_targets.append("include:" + ",".join(
                (child.text or "").strip()
                for child in include
                if strip_namespace(child.tag) in {"uri", "name"} and child.text
            ))

    indent_xml(root_copy)
    output_tree = ET.ElementTree(root_copy)
    args.output_world.parent.mkdir(parents=True, exist_ok=True)
    output_tree.write(args.output_world, encoding="utf-8", xml_declaration=True)

    print(f"Input: {args.input_world}")
    print(f"Output: {args.output_world}")
    print(f"Updated visual/collision scale tags: {updated_count}")
    if scaled_targets:
        print("Scaled targets:")
        for target in scaled_targets:
            print(f"  - {target}")
    else:
        print("Scaled targets: none found in this world file")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
