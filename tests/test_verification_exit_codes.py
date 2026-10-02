import ast
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SENSORS = ("BarometerSensor", "DVLsensor", "UW_Camera", "ImagingSonarSensor")


def run_checks(
    filename: str,
    monkeypatch: pytest.MonkeyPatch,
    failing: str | None,
    moving: bool = True,
) -> tuple[int, bool, dict]:
    """Execute the actual check/report blocks with SDK doubles; no simulator is started."""
    sensor = SimpleNamespace(
        get_pressure=lambda: 123239.5,
        attachDVL=lambda **kwargs: None,
        add_debug_lines=lambda: None,
        get_beam_paths=lambda: list(range(4)),
        set_focal_length=lambda *args: None,
        set_clipping_range=lambda *args: None,
        get_resolution=lambda: [640, 480],
        get_range=lambda: [0.2, 3.0],
        get_fov=lambda: [130.0, 20.0],
    )
    scenario = SimpleNamespace(
        _baro_reading=123239.5,
        setup_scenario=lambda **kwargs: None,
        setup_waypoints=lambda **kwargs: None,
        update_scenario=lambda *args: None,
        teardown_scenario=lambda: None,
    )

    def constructor(name: str):
        def create(*args, **kwargs):
            if name == failing:
                raise RuntimeError(f"injected {name} failure")
            return scenario if name == "Scenario" else sensor

        return create

    namespace = {
        "sys": sys,
        "np": np,
        "robot_prim_path": "/World/rob",
        "euler_angles_to_quat": lambda *args, **kwargs: np.array([1, 0, 0, 0]),
    }
    for name in SENSORS:
        namespace[name] = constructor(name)
        full_name = f"isaacsim.oceansim.sensors.{name}"
        module = ModuleType(full_name)
        setattr(module, name, namespace[name])
        monkeypatch.setitem(sys.modules, full_name, module)
    module_name = "isaacsim.oceansim.modules.SensorExample_python.scenario"
    module = ModuleType(module_name)
    module.MHL_Sensor_Example_Scenario = constructor("Scenario")
    monkeypatch.setitem(sys.modules, module_name, module)
    positions = iter([(0, 0, 0), (0.1 if moving else 0, 0, 0)])
    namespace["rob"] = SimpleNamespace(
        GetAttribute=lambda *args: SimpleNamespace(Get=lambda: next(positions))
    )
    closed = []
    namespace["app"] = SimpleNamespace(close=lambda: closed.append(True))
    namespace["open"] = lambda *args, **kwargs: io.StringIO()

    tree = ast.parse((ROOT / filename).read_text(), filename=filename)
    first_check = next(
        i for i, node in enumerate(tree.body) if isinstance(node, ast.Try)
    )
    setup = [
        node
        for node in tree.body[:first_check]
        if isinstance(node, ast.Assign)
        and any(
            isinstance(t, ast.Name) and t.id in {"results", "failed_checks"}
            for t in node.targets
        )
    ]
    checks = ast.Module(body=setup + tree.body[first_check:], type_ignores=[])
    try:
        exec(compile(checks, filename, "exec"), namespace)
    except SystemExit as exc:
        return int(exc.code or 0), bool(closed), namespace
    return 0, bool(closed), namespace


@pytest.mark.parametrize("failing", [None, *SENSORS])
def test_headless_exit_reports_sensor_failures(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture, failing: str | None
) -> None:
    status, closed, _ = run_checks("test_oceansim_headless.py", monkeypatch, failing)
    output = capsys.readouterr().out
    assert closed
    assert status == (0 if failing is None else 1)
    if failing:
        assert "tested successfully" not in output


@pytest.mark.parametrize("failing", [None, *SENSORS, "Scenario"])
def test_phase3_exit_reports_sensor_and_scenario_failures(
    monkeypatch: pytest.MonkeyPatch, failing: str | None
) -> None:
    status, closed, _ = run_checks("verify_phase3.py", monkeypatch, failing)
    assert closed
    assert status == (0 if failing is None else 1)


def test_phase3_rejects_stationary_waypoint_run(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    status, closed, namespace = run_checks(
        "verify_phase3.py", monkeypatch, None, moving=False
    )
    assert closed
    assert status == 1
    assert any(r.startswith("[FAIL] Waypoint following") for r in namespace["results"])
