"""Truth-map isolation: navigation filters must never see the truth DSM."""
import ast
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
FILTER_DIR = ROOT / "trn" / "filters"
FORBIDDEN_MODULES = ("trn.truth", "trn.laser.altimeter", "trn.atmosphere", "trn.trajectory", "trn.experiments",
                     "rasterio")
FORBIDDEN_IDENTIFIERS = ("truth", "dsm")       # variable / attribute names
FORBIDDEN_LITERALS = ("dsm", ".tif", ".npy", "Data/")   # file-like string literals


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module)
    return mods


def _closure(start: set[str]) -> set[str]:
    """Transitive closure of trn.* imports starting from the filter modules."""
    seen, todo = set(), set(start)
    while todo:
        m = todo.pop()
        if m in seen:
            continue
        seen.add(m)
        if m.startswith("trn"):
            p = ROOT / (m.replace(".", "/") + ".py")
            if not p.exists():
                p = ROOT / m.replace(".", "/") / "__init__.py"
            if p.exists():
                todo |= _imports(p)
    return seen


@pytest.mark.parametrize("path", sorted(FILTER_DIR.glob("*.py")), ids=lambda p: p.name)
def test_filter_modules_do_not_import_truth(path):
    closure = _closure(_imports(path))
    bad = [m for m in closure if m.startswith(FORBIDDEN_MODULES)]
    assert not bad, f"{path.name} (transitively) imports {bad}"


@pytest.mark.parametrize("path", sorted(FILTER_DIR.glob("*.py")), ids=lambda p: p.name)
def test_filter_source_has_no_truth_references(path):
    tree = ast.parse(path.read_text())
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | \
            {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)} | \
            {a.arg for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) for a in n.args.args}
    for s in FORBIDDEN_IDENTIFIERS:
        assert not any(s in x.lower() for x in names), f"identifier containing '{s}' in {path.name}"
    docs = {id(n.body[0].value) for n in ast.walk(tree)
            if isinstance(n, (ast.Module, ast.FunctionDef, ast.ClassDef)) and n.body
            and isinstance(n.body[0], ast.Expr) and isinstance(n.body[0].value, ast.Constant)}
    lits = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and id(n) not in docs]
    for s in FORBIDDEN_LITERALS:
        assert not any(s.lower() in x.lower() for x in lits), f"literal containing '{s}' in {path.name}"


def test_importing_filters_does_not_load_truth():
    code = "import sys, trn.filters.mpf, trn.filters.tercom; " \
           "bad=[m for m in sys.modules if m.startswith(('trn.truth','trn.laser.altimeter','rasterio'))]; print(bad)"
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=ROOT).stdout.strip()
    assert out == "[]", out


def test_filter_rejects_non_onboard_map():
    from trn.filters.tercom import Tercom
    from trn.terrain.grid import Grid
    with pytest.raises(TypeError):
        Tercom(Grid(np.zeros((3, 3)), 0.0, 0.0, 1.0), {})
