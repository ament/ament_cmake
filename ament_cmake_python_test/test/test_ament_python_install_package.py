from pathlib import Path
import os
import filecmp
import shutil

INSTALL_DIR = Path(os.environ["TEST_PACKAGE_INSTALL_DIR"])
TEST_DIR = Path(__file__).parent
AMENT_PYTHON_TEST_PACKAGE = "ament_python_test_package"
AMENT_PYTHON_TEST_PACKAGE_OVERLAY = AMENT_PYTHON_TEST_PACKAGE + "_overlay"

def test_ament_python_test_package() -> None:
    assert not filecmp.dircmp(
        TEST_DIR / AMENT_PYTHON_TEST_PACKAGE, 
        INSTALL_DIR / AMENT_PYTHON_TEST_PACKAGE
    ).diff_files

def test_ament_python_test_package_with_overlay(tmpdir) -> None:
    shutil.copytree(TEST_DIR / AMENT_PYTHON_TEST_PACKAGE, tmpdir, dirs_exist_ok=True)
    shutil.copytree(TEST_DIR / AMENT_PYTHON_TEST_PACKAGE_OVERLAY, tmpdir, dirs_exist_ok=True)
    assert not filecmp.dircmp(tmpdir, INSTALL_DIR / AMENT_PYTHON_TEST_PACKAGE_OVERLAY).diff_files
    assert not filecmp.dircmp(
        tmpdir / "subdir", 
        INSTALL_DIR / AMENT_PYTHON_TEST_PACKAGE_OVERLAY / "subdir"
    ).diff_files
