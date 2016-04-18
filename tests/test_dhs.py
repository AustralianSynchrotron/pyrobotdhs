import pytest
from mock import MagicMock, call

from pyrobotdhs import RobotDHS
from pyrobotdhs.dhs import HOLDER_TYPE_CASSETTE, HOLDER_TYPE_ADAPTOR


@pytest.fixture
def dhs():
    return RobotDHS(dcss='0.0.0.0', robot=MagicMock())


def test_robot_config(dhs):
    dhs.robot_config_test = MagicMock()
    dhs.robot_config('operation', 'test', 1, 2, 3)
    assert dhs.robot_config_test.call_args == (('operation', 1, 2, 3),)


@pytest.mark.parametrize('start,output', [(0, 1), (1, 0)])
def test_robot_config_hw_output_switch_for_gripper(dhs, start, output):
    mock_robot = dhs.robot
    mock_operation = MagicMock()
    mock_robot.gripper_command = start
    mock_robot.set_gripper.return_value = None
    dhs.robot_config_hw_output_switch(mock_operation, '1')
    assert mock_robot.set_gripper.call_args[0][0] == output


def test_robot_config_reset_cassette(dhs):
    dhs.robot_config_reset_cassette(MagicMock())
    expected_ports = {'left': [1] * 96, 'middle': [1] * 96, 'right': [1] * 96}
    assert dhs.robot.reset_ports.call_args[0][0] == expected_ports


def test_robot_config_set_index_state_left_column_A(dhs):
    dhs.robot_config_set_index_state(MagicMock(), '1', '8', 'b')
    expected_ports = {'left': [1] * 8 + [0] * 88,
                      'middle': [0] * 96,
                      'right': [0] * 96}
    assert dhs.robot.reset_ports.call_args[0][0] == expected_ports


def test_robot_config_set_index_state_middle_adaptor_mB1(dhs):
    dhs.robot_config_set_index_state(MagicMock(), '114', '1', 'b')
    expected_ports = {'left': [0] * 96,
                      'middle': [0] * 16 + [1] + [0] * 79,
                      'right': [0] * 96}
    assert dhs.robot.reset_ports.call_args[0][0] == expected_ports


def test_system_error_message_updates_dcss(dhs):
    dhs.send_xos3 = MagicMock()
    dhs.on_system_error_message('Bad bad happened')
    assert dhs.send_xos3.call_args == call('htos_log error Bad bad happened')


def test_system_error_message_does_not_update_dcss_when_ok(dhs):
    dhs.send_xos3 = MagicMock()
    dhs.on_system_error_message('OK')
    assert dhs.send_xos3.call_args_list == []


def test_port_tuple_to_str_for_cassettes(dhs):
    dhs.robot.configure_mock(
        holder_types={'left': HOLDER_TYPE_CASSETTE}
    )
    s = dhs.port_tuple_to_str(('left', 16))
    assert s == 'l 1 C'


def test_port_tuple_to_str_for_adaptors(dhs):
    dhs.robot.configure_mock(
        holder_types={'left': HOLDER_TYPE_ADAPTOR}
    )
    s = dhs.port_tuple_to_str(('left', 16))
    assert s == 'l 1 B'


def test_send_set_state_string(dhs):
    dhs.send_xos3 = MagicMock()
    # TODO: Test setting current_sample
    dhs.robot.configure_mock(
        closest_point=18,
        ln2_level=0,
        holder_types={'left': HOLDER_TYPE_CASSETTE},
        sample_locations={
            'goniometer': (),
            'cavity': ('left', 0),
            'picker': None,
            'placer': None,
        }
    )
    dhs.send_set_state_string()
    assert dhs.send_xos3.call_args == call(
        'htos_set_string_completed robot_state normal '
        '{on gonio} {in cradle} '
        'P18 '
        'no '
        '{} '
        '0 0 0 '
        '0 '
        '0 0 '
        '{l 1 A} '
        '{invalid} {invalid} '
        '0 0 0 0'
    )


def test_robot_config_probe(dhs):
    dhs.robot.probe.return_value = None
    dhs.operation_complete = MagicMock()
    mock_operation = MagicMock()
    ports = ['1'] * (96 + 1) + ['0'] * (96 + 1) * 2
    dhs.robot_config_probe(mock_operation, *ports)
    expected_spec = {'left': [1] * 96, 'middle': [0] * 96, 'right': [0] * 96}
    assert dhs.robot.probe.call_args[0][0] == expected_spec


def test_setting_foreground_operation(dhs):
    dhs.send_set_status_string = MagicMock()
    dhs.foreground_operation = 'go'
    assert dhs.send_set_status_string.called is True


def test_prepare_mount_crystal(dhs):
    dhs.robot.prepare_for_mount.return_value = None
    dhs.operation_complete = MagicMock()
    mock_operation = MagicMock()  # TODO: Make a fixture
    dhs.prepare_mount_crystal(mock_operation, 'r', '6', 'A', '0', '0', '0', '0')
    assert mock_operation.operation_update.call_args == call('OK to prepare')
    assert dhs.robot.prepare_for_mount.called is True


def test_send_calibration_timestamps(dhs):
    dhs.robot.configure_mock(
        last_toolset_calibration='2016/02/08 11:39:12',
        last_left_calibration=None,
        last_middle_calibration=None,
        last_right_calibration=None,
        last_goniometer_calibration=None,
    )
    dhs.send_xos3 = MagicMock()
    dhs.send_calibration_timestamps()
    expected_msg = ('htos_set_string_completed ts_robot_cal normal '
                    '{2016/02/08 11:39:12} {} {} {} {}')
    assert dhs.send_xos3.call_args == call(expected_msg)
