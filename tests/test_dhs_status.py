import pytest
from pyrobotdhs import RobotDHS


class MockRobot():
    def __init__(self):
        self.safety_gate = 0
        self.at_home = 1
        self.foreground_done = 1


@pytest.fixture
def dhs():
    dhs = RobotDHS(dcss='0.0.0.0', robot=MockRobot())
    dhs._needs_clear = False
    return dhs


def test_dhs_status_ok(dhs):
    assert dhs.status == 0


def test_dhs_status_needs_clear(dhs):
    dhs._needs_clear = True
    assert dhs.status == RobotDHS.STATUS_NEED_CLEAR


def test_dhs_status_handles_safety_gate(dhs):
    dhs.robot.safety_gate = 1
    assert dhs.status == RobotDHS.STATUS_REASON_SAFEGUARD


def test_dhs_status_combinations(dhs):
    dhs._needs_clear = 1
    dhs.robot.safety_gate = 1
    assert dhs.status == (RobotDHS.STATUS_NEED_CLEAR |
                          RobotDHS.STATUS_REASON_SAFEGUARD)


def test_dhs_status_not_at_home(dhs):
    dhs.robot.at_home = 0
    assert dhs.status == RobotDHS.STATUS_REASON_NOT_HOME
