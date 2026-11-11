import shutil
import subprocess
from pathlib import Path

import pytest

TXN_DIR = Path(__file__).resolve().parents[2] / "firmware" / "txn"
CFLAGS = ["-std=c11", "-Wall", "-Wextra", "-Wpedantic", "-Wconversion", "-Wsign-conversion", "-Werror", "-g"]
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _build(compiler: str, out: Path, extra: list[str]) -> subprocess.CompletedProcess:
    cmd = [compiler, *CFLAGS, *extra, f"-I{TXN_DIR}", str(TXN_DIR / "txn.c"), str(TXN_DIR / "tests" / "test_txn.c"), "-o", str(out)]
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def test_txn_c_unit_tests(tmp_path):
    compiler = shutil.which("gcc") or shutil.which("cc")
    if compiler is None:
        pytest.skip("no host C compiler")

    exe = tmp_path / "test_txn"
    build = _build(compiler, exe, SANITIZERS)
    if build.returncode != 0:
        # Sanitizer runtimes are not installed everywhere.
        build = _build(compiler, exe, [])
    assert build.returncode == 0, build.stderr

    run = subprocess.run([str(exe)], check=False, capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip().endswith("OK: 0 failure(s)"), run.stdout
