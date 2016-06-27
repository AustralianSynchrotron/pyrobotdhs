import pytest
from unittest.mock import MagicMock, call

from aspyrobotmx.codes import (HolderType, PortState, RobotStatus, DumbbellState,
                               SampleState)

from pyrobotdhs import RobotDHS


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


def test_robot_config_hw_output_switch_for_heater(dhs):
    mock_robot = dhs.robot
    mock_robot.heater_command = 1
    dhs.robot_config_hw_output_switch(MagicMock(), '14')
    assert mock_robot.set_heater.call_args[0][0] == 0


def test_robot_config_hw_output_switch_for_heater_air(dhs):
    mock_robot = dhs.robot
    mock_robot.heater_air_command = 1
    dhs.robot_config_hw_output_switch(MagicMock(), '13')
    assert mock_robot.set_heater_air.call_args[0][0] == 0


def test_robot_config_reset_cassette(dhs):
    dhs.robot_config_reset_cassette(MagicMock())
    assert dhs.robot.reset_holders.call_args[0] == (['left', 'middle', 'right'],)


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


def test_robot_config_set_port_state(dhs):
    dhs.robot_config_set_port_state(MagicMock(), 'lX0', 'u')
    assert dhs.robot.reset_holders.call_args[0] == (['left'],)


def test_system_error_message_updates_dcss(dhs):
    dhs.send_xos3 = MagicMock()
    dhs.on_system_error_message('Bad bad happened')
    assert dhs.send_xos3.call_args == call('htos_log error robot Bad bad happened')


def test_system_error_message_does_not_update_dcss_when_ok(dhs):
    dhs.send_xos3 = MagicMock()
    dhs.on_system_error_message('OK')
    assert dhs.send_xos3.call_args_list == []


@pytest.mark.parametrize('callback', ['on_last_toolset_calibration',
                                      'on_last_left_calibration',
                                      'on_last_middle_calibration',
                                      'on_last_right_calibration',
                                      'on_last_goniometer_calibration'])
def test_calibration_time_updates_send_timestamp_message(dhs, callback):
    dhs.send_calibration_timestamps = MagicMock()
    getattr(dhs, callback)('2016/05/20 10:20:30')
    assert dhs.send_calibration_timestamps.called is True


def test_port_tuple_to_str_for_cassettes(dhs):
    dhs.robot.configure_mock(
        holder_types={'left': HolderType.normal}
    )
    s = dhs.port_tuple_to_str(('left', 16))
    assert s == 'l 1 C'


def test_port_tuple_to_str_for_adaptors(dhs):
    dhs.robot.configure_mock(
        holder_types={'left': HolderType.superpuck}
    )
    s = dhs.port_tuple_to_str(('left', 16))
    assert s == 'l 1 B'


def test_send_set_state_string(dhs):
    dhs.send_xos3 = MagicMock()
    # TODO: Test setting current_sample
    dhs.robot.configure_mock(
        dumbbell_state=1,
        closest_point=18,
        ln2_level=0,
        holder_types={'left': HolderType.normal},
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
        '{on tong} {in_cradle} '
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


def test_send_set_state_string_with_sample_on_goni(dhs):
    dhs.send_xos3 = MagicMock()
    dhs.robot.configure_mock(
        dumbbell_state=1,
        closest_point=18,
        ln2_level=0,
        holder_types={'left': HolderType.normal},
        sample_locations={
            'goniometer': ('left', 0),
            'cavity': None,
            'picker': None,
            'placer': None,
        }
    )
    dhs.send_set_state_string()
    assert dhs.send_xos3.call_args == call(
        'htos_set_string_completed robot_state normal '
        '{on gonio} {in_cradle} '
        'P18 '
        'no '
        '{l 1 A} '
        '0 0 0 '
        '1 '
        '0 0 '
        '{invalid} '
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


def test_set_robot_cassette_string(dhs):
    dhs.robot.configure_mock(
        sample_locations={
            'goniometer': ['left', 0],
        },
        port_states={
            'left': [PortState.full] * 96,
            'middle': [PortState.empty] * 96,
            'right': [PortState.unknown] * 96,
        },
        holder_types={
            'left': HolderType.normal,
            'middle': HolderType.superpuck,
            'right': HolderType.unknown,
        }
    )
    dhs.send_xos3 = MagicMock()
    dhs.send_set_robot_cassette_string()
    expected_msg = (
        'htos_set_string_completed robot_cassette normal '
        '{left} {middle} {right}'
    ).format(
        left=' '.join(['1'] + ['m'] + ['1'] * 95),
        middle=' '.join(['3'] + ['0'] * 96),
        right=' '.join(['u'] + ['u'] * 96),
    )
    assert dhs.send_xos3.call_args == call(expected_msg)


def test_send_set_output_string(dhs):
    dhs.robot.configure_mock(
        gripper_command=1,
        lid_command=1,
        heater_command=1,
        heater_air_command=1,
    )
    dhs.send_xos3 = MagicMock()
    dhs.send_set_output_string()
    expected_msg = ('htos_set_string_completed robot_output normal '
                    '0 1 0 1 0 0 0 0 0 0 0 0 0 1 1 0')
    assert dhs.send_xos3.call_args == call(expected_msg)


def test_send_set_input_string(dhs):
    dhs.robot.configure_mock(
        gripper_open=1,
        gripper_closed=1,
        lid_closed=1,
        lid_open=1,
        heater_hot=1,
    )
    dhs.send_xos3 = MagicMock()
    dhs.send_set_input_string()
    expected_msg = ('htos_set_string_completed robot_input normal '
                    '0 0 0 0 0 0 0 0 1 1 0 1 1 1 0 0')
    assert dhs.send_xos3.call_args == call(expected_msg)


def test_send_set_status_string(dhs):
    dhs.send_xos3 = MagicMock()
    status = (RobotStatus.need_clear | RobotStatus.reason_collision |
              RobotStatus.need_cal_cassette)
    dhs.robot.configure_mock(
        status=status,
        current_task='idle',
        task_message='all good',
        task_progress='1 of 10',
        holder_types={'left': HolderType.normal},
        sample_locations={'goniometer': ['left', 0]},
        pins_lost=123,
        pins_mounted=456,
    )
    dhs.send_set_status_string()
    expected_msg = ('htos_set_string_completed robot_status '
                    'normal '
                    'status: 32777 '
                    'need_reset: 0 '
                    'need_cal: 1 '
                    'state: {idle} '
                    'warning: {} '
                    'cal_msg: {all good} '
                    'cal_step: {1 of 10} '
                    'mounted: {l 1 A} '
                    'pin_lost: 123 '
                    'pin_mounted: 456 '
                    'manual_mode: 0 '
                    'need_mag_cal: 0 '
                    'need_cas_cal: 1 '
                    'need_clear: 1')
    assert dhs.send_xos3.call_args == call(expected_msg)


@pytest.mark.parametrize('sample_locations,value', [
    ({}, 'no'),
    ({'cavity': ['left', 0]}, 'on tong'),
    ({'picker': ['left', 0]}, 'on picker'),
    ({'placer': ['left', 0]}, 'on placer'),
    ({'goniometer': ['left', 0]}, 'on gonio'),
])
def test_sample_state(dhs, sample_locations, value):
    dhs.robot.configure_mock(sample_locations=sample_locations)
    assert dhs.sample_state == value


@pytest.mark.parametrize('code,value', [
    (None, 'bad'),
    (int(DumbbellState.unknown), 'unknown'),
    (int(DumbbellState.in_cradle), 'in_cradle'),
])
def test_dumbbell_state(dhs, code, value):
    dhs.robot.configure_mock(dumbbell_state=code)
    assert dhs.dumbbell_state == value


@pytest.mark.parametrize('level,expected_str', [
    (None, 'wrong'), (0, 'no'), (1, 'yes'),
])
def test_ln2_property(dhs, level, expected_str):
    dhs.robot.configure_mock(ln2_level=level)
    assert dhs.ln2 == expected_str


def test_robot_config_set_mounted(dhs):
    mock_robot = dhs.robot
    dhs.robot_config_set_mounted(MagicMock(), 'mJ2')
    expected_args = ('middle', 'J', 2, SampleState.goniometer)
    assert mock_robot.set_sample_state.call_args[0] == expected_args


def test_robot_config_set_mounted_with_two_digit_port(dhs):
    mock_robot = dhs.robot
    dhs.robot_config_set_mounted(MagicMock(), 'mJ12')
    expected_args = ('middle', 'J', 12, SampleState.goniometer)
    assert mock_robot.set_sample_state.call_args[0] == expected_args
