"""Mid-Air adapter behaviour that does not need the real files."""
import numpy as np
import pytest

from src.data.midair import (
    ENV_VAR,
    MidAirConfig,
    MidAirDataNotFound,
    _to_wxyz,
    find_sensor_file,
    load_midair_trajectory,
    normalize_trajectory_name,
    resolve_data_root,
)


def test_missing_root_explains_how_to_configure(monkeypatch):
    monkeypatch.delenv(ENV_VAR, raising=False)
    with pytest.raises(MidAirDataNotFound, match=ENV_VAR) as exc:
        load_midair_trajectory(MidAirConfig())
    assert "--synthetic" in str(exc.value)


def test_nonexistent_root(monkeypatch, tmp_path):
    monkeypatch.delenv(ENV_VAR, raising=False)
    with pytest.raises(MidAirDataNotFound, match="does not exist"):
        resolve_data_root(str(tmp_path / "nope"), None)


def test_root_precedence(monkeypatch, tmp_path):
    a, b, c = (tmp_path / x for x in "abc")
    for d in (a, b, c):
        d.mkdir()
    monkeypatch.setenv(ENV_VAR, str(b))
    assert resolve_data_root(str(a), str(c)) == a
    assert resolve_data_root(None, str(c)) == b
    monkeypatch.delenv(ENV_VAR)
    assert resolve_data_root(None, str(c)) == c


def test_empty_root_lists_what_was_tried(tmp_path):
    with pytest.raises(MidAirDataNotFound, match="Tried"):
        find_sensor_file(tmp_path, MidAirConfig())


def test_zipped_sensor_file_asks_to_unzip(tmp_path):
    folder = tmp_path / "Kite_training" / "sunny"
    folder.mkdir(parents=True)
    (folder / "sensor_records.zip").write_bytes(b"")
    with pytest.raises(MidAirDataNotFound, match="Unzip"):
        find_sensor_file(tmp_path, MidAirConfig())


def test_trajectory_name():
    assert normalize_trajectory_name(3) == "trajectory_0003"
    assert normalize_trajectory_name("2003") == "trajectory_2003"
    assert normalize_trajectory_name("trajectory_0000") == "trajectory_0000"


def test_quaternion_reorder():
    q = np.array([[0.1, 0.2, 0.3, 0.9]])  # x, y, z, w
    np.testing.assert_array_equal(_to_wxyz(q, "xyzw"), [[0.9, 0.1, 0.2, 0.3]])
    np.testing.assert_array_equal(_to_wxyz(q, "wxyz"), q)
    with pytest.raises(ValueError):
        _to_wxyz(q, "zyxw")


def test_config_from_dict():
    cfg = MidAirConfig.from_dict({"trajectory": 5, "keys": {"position": "gt/pos"},
                                  "conventions": {"quaternion_order": "xyzw"}})
    assert cfg.trajectory == "trajectory_0005"
    assert cfg.keys.position == "gt/pos"
    assert cfg.keys.attitude == "groundtruth/attitude"
    assert cfg.conventions.quaternion_order == "xyzw"
