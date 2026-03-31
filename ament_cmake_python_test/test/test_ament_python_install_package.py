from pathlib import Path
import filecmp
import os
import shutil

INSTALL_DIR = Path(os.environ["TEST_PACKAGE_INSTALL_DIR"])
TEST_DIR = Path(__file__).parent
PKG = "ament_python_test_package"
PKG_OVERLAY = PKG + "_overlay"
PKG_NO_COMPILE = PKG + "_no_compile"
PKG_VERSIONED = PKG + "_versioned"
PKG_GENERATED = PKG + "_generated"


def _assert_dirs_match(expected, actual):
    """Recursively assert two directories have identical contents."""
    cmp = filecmp.dircmp(expected, actual)
    assert not cmp.diff_files
    assert not cmp.left_only
    assert not cmp.right_only
    for subdir in cmp.common_dirs:
        if subdir == "__pycache__":
            continue
        _assert_dirs_match(Path(expected) / subdir, Path(actual) / subdir)


def test_single_package():
    """Single package dir installs all files correctly."""
    _assert_dirs_match(TEST_DIR / PKG, INSTALL_DIR / PKG)


def test_overlay_merges_files(tmp_path):
    """Two package dirs merge correctly, last dir wins on conflicts."""
    merged = tmp_path / "merged"
    shutil.copytree(TEST_DIR / PKG, merged)
    shutil.copytree(TEST_DIR / PKG_OVERLAY, merged, dirs_exist_ok=True)
    _assert_dirs_match(merged, INSTALL_DIR / PKG_OVERLAY)


def test_skip_compile():
    """SKIP_COMPILE prevents .pyc generation."""
    pkg_dir = INSTALL_DIR / PKG_NO_COMPILE
    assert pkg_dir.exists()
    assert not list(pkg_dir.rglob("*.pyc"))


def test_default_compiles():
    """Default behavior produces .pyc files."""
    pkg_dir = INSTALL_DIR / PKG
    assert list(pkg_dir.rglob("*.pyc"))


def test_egg_info():
    """Egg-info is generated."""
    egg_dirs = list(INSTALL_DIR.glob("*.egg-info"))
    assert egg_dirs


def test_explicit_version():
    """Explicit VERSION propagates to egg-info."""
    egg_dirs = list(INSTALL_DIR.glob("*versioned*1.2.3*.egg-info"))
    assert len(egg_dirs) == 1


def test_depends_generates_files():
    """DEPENDS ensures build-time generated files are copied by sync."""
    pkg_dir = INSTALL_DIR / PKG_GENERATED
    assert pkg_dir.exists()
    assert (pkg_dir / "_generated_marker.py").exists()
