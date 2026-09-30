"""Validate Fixposition integration setup."""

import xml.etree.ElementTree as ET
from pathlib import Path


def _load_urdf():
    urdf_path = (
        Path(__file__).resolve().parents[2] / "terrascout_description" / "urdf" / "scout.urdf.xacro"
    )
    return ET.parse(urdf_path)


def test_vrtk_link_exists_as_root():
    tree = _load_urdf()
    root = tree.getroot()
    links = [link.get("name") for link in root.findall("link")]
    assert "vrtk_link" in links, "vrtk_link must exist in URDF"

    # vrtk_link should be a parent (not a child of any joint)
    child_links = set()
    for j in root.findall("joint"):
        child = j.find("child")
        assert child is not None
        child_links.add(child.get("link"))
    assert "vrtk_link" not in child_links, "vrtk_link should be the root (not a child)"


def test_vrtk_to_base_footprint_joint():
    tree = _load_urdf()
    root = tree.getroot()
    for joint in root.findall("joint"):
        if joint.get("name") == "vrtk_joint":
            parent = joint.find("parent")
            child = joint.find("child")
            assert parent is not None
            assert child is not None
            assert parent.get("link") == "vrtk_link"
            assert child.get("link") == "base_footprint"
            assert joint.get("type") == "fixed"
            return
    raise AssertionError("vrtk_joint not found in URDF")


def test_tf_chain_completeness():
    """Verify the full chain: vrtk_link -> base_footprint -> base_link exists."""
    tree = _load_urdf()
    root = tree.getroot()

    joints = {}
    for joint in root.findall("joint"):
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        assert parent_el is not None and child_el is not None
        parent_link = parent_el.get("link")
        child_link = child_el.get("link")
        assert parent_link is not None and child_link is not None
        joints[child_link] = parent_link

    # Walk from base_link up to vrtk_link
    current = "base_link"
    chain = [current]
    while current in joints:
        current = joints[current]
        chain.append(current)

    assert chain == ["base_link", "base_footprint", "vrtk_link"], (
        f"Expected chain base_link->base_footprint->vrtk_link, got {chain}"
    )
