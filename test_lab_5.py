#!/usr/bin/env python3
"""
Autograding validation for Lab 5: MoveIt 2 Motion Planning (Kinova Gen3 Lite).
Run with: pytest test_lab_5.py -v
"""

import ast
import os
import warnings

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".bmp", ".webp"}
README_STARTER_FINGERPRINT = "Update this README"
PKG_DIR = os.path.join("ros2_ws", "src", "lab05_moveit")


def _find_in_package(name):
    for root, dirs, files in os.walk(PKG_DIR):
        if name in files:
            return os.path.join(root, name)
    return os.path.join(PKG_DIR, name)


def _get_images(docs_dir="docs"):
    if not os.path.isdir(docs_dir):
        return []
    return [
        f for f in os.listdir(docs_dir)
        if os.path.splitext(f)[1].lower() in IMAGE_EXTENSIONS
    ]


def _check_syntax(path):
    with open(path, "r", errors="replace") as f:
        source = f.read()
    ast.parse(source, filename=path)


# ── Required files (hard fail) ──────────────────────────────


def test_readme_exists():
    assert os.path.isfile("README.md"), "README.md not found"


def test_readme_not_empty():
    assert os.path.getsize("README.md") > 0, "README.md is empty"


def test_readme_updated():
    with open("README.md", "r", errors="replace") as f:
        content = f.read()
    assert README_STARTER_FINGERPRINT not in content, (
        "README.md still contains the starter template text. "
        "Please update it with your name, NetID, and instructions for running your code."
    )


def test_docs_directory_exists():
    assert os.path.isdir("docs"), "docs/ directory not found"


def test_package_directory_exists():
    assert os.path.isdir(PKG_DIR), f"ROS 2 package not found at {PKG_DIR}"


def test_setup_py_exists():
    assert os.path.isfile(os.path.join(PKG_DIR, "setup.py")), "setup.py not found"


def test_package_xml_exists():
    assert os.path.isfile(os.path.join(PKG_DIR, "package.xml")), "package.xml not found"


def test_motion_planner_exists():
    path = _find_in_package("motion_planner.py")
    assert os.path.isfile(path), "motion_planner.py not found in package"


# ── Python syntax (hard fail) ───────────────────────────────


def test_motion_planner_syntax():
    _check_syntax(_find_in_package("motion_planner.py"))


# ── Optional files ───────────────────────────────────────────


def test_optional_m5_detour():
    path = _find_in_package("m5_detour.py")
    if os.path.isfile(path):
        print("m5_detour.py found (stretch goal)")
        _check_syntax(path)
    else:
        print("m5_detour.py not found (optional stretch goal)")


# ── Screenshots (warnings) ──────────────────────────────────


def test_screenshot_count():
    images = _get_images()
    print(f"\nFound {len(images)} image(s) in docs/:")
    for img in sorted(images):
        print(f"  - {img}")
    if len(images) < 5:
        warnings.warn(f"Expected at least 5 screenshots, found {len(images)}")


# ── Git hygiene (warnings) ──────────────────────────────────


def test_gitignore_exists():
    if not os.path.isfile(".gitignore"):
        warnings.warn(".gitignore not found — build/, install/, log/ should be excluded")


def test_no_build_artifacts():
    for d in ["build", "install", "log"]:
        for base in [".", "ros2_ws"]:
            path = os.path.join(base, d)
            if os.path.isdir(path):
                warnings.warn(f"'{path}' is committed — should be in .gitignore")
