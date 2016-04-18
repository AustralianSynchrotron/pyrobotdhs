from functools import partial
from threading import Thread

from dcss import Server as DHS


HOLDER_TYPE_UNKNOWN = 'u'
HOLDER_TYPE_CASSETTE = '1'
HOLDER_TYPE_CALIBRATION_CASSETTE = '2'
HOLDER_TYPE_ADAPTOR = '3'
HOLDER_TYPE_BAD = 'X'

PORT_STATE_FULL = -1
PORT_STATE_UNKNOWN = 0
PORT_STATE_EMPTY = 1
PORT_STATE_ERROR = 2

SAMPLES_PER_POSITION = 96

PORT_STATE_LOOKUP = {
    PORT_STATE_UNKNOWN: 'u',
    PORT_STATE_EMPTY: '0',
    PORT_STATE_FULL: '1',
    PORT_STATE_ERROR: 'b',
}
PORT_STATE_REVERSE_LOOKUP = {v: k for k, v in PORT_STATE_LOOKUP.items()}


class RobotDHS(DHS):

    OUTPUT_GRIPPER = 1
    OUTPUT_LID = 3
    OUTPUT_DRY_AIR = 13
    OUTPUT_HEATER = 14

    STATUS_NEED_ALL = 0x0000007f
    STATUS_NEED_CAL_ALL = 0x0000003C
    STATUS_NEED_CLEAR = 0x00000001
    STATUS_NEED_RESET = 0x00000002
    STATUS_NEED_CAL_MAGNET = 0x00000004
    STATUS_NEED_CAL_CASSETTE = 0x00000008
    STATUS_NEED_CAL_GONIO = 0x00000010
    STATUS_NEED_CAL_BASIC = 0x00000020
    STATUS_NEED_USER_ACTION = 0x00000040
    STATUS_REASON_ALL = 0x0fffff80
    STATUS_REASON_PORT_JAM = 0x00000080
    STATUS_REASON_ESTOP = 0x00000100
    STATUS_REASON_SAFEGUARD = 0x00000200
    STATUS_REASON_NOT_HOME = 0x00000400
    STATUS_REASON_CMD_ERROR = 0x00000800
    STATUS_REASON_LID_JAM = 0x00001000
    STATUS_REASON_GRIPPER_JAM = 0x00002000
    STATUS_REASON_LOST_MAGNET = 0x00004000
    STATUS_REASON_COLLISION = 0x00008000
    STATUS_REASON_INIT = 0x00010000
    STATUS_REASON_TOLERANCE = 0x00020000
    STATUS_REASON_LN2LEVEL = 0x00040000
    STATUS_REASON_HEATER_FAIL = 0x00080000
    STATUS_REASON_CASSETTE = 0x00100000
    STATUS_REASON_PIN_LOST = 0x00200000
    STATUS_REASON_WRONG_STATE = 0x00400000
    STATUS_REASON_BAD_ARG = 0x00800000
    STATUS_REASON_SAMPLE_IN_PORT = 0x01000000
    STATUS_REASON_ABORT = 0x02000000
    STATUS_REASON_UNREACHABLE = 0x04000000
    STATUS_REASON_EXTERNAL = 0x08000000
    STATUS_IN_ALL = 0xf0000000
    STATUS_IN_RESET = 0x10000000
    STATUS_IN_CALIBRATION = 0x20000000
    STATUS_IN_TOOL = 0x40000000
    STATUS_IN_MANUAL = 0x80000000

    def __init__(self, dcss, robot):

        super(RobotDHS, self).__init__('robot', dcss)
        self.robot = robot
        self.robot.delegate = self

        self._foreground_operation = None
        self._needs_clear = True

    def setup(self):
        # Start DHS.loop to process incoming dcss messages
        self.recv_loop_thread = Thread(target=self.loop, daemon=True)
        self.recv_loop_thread.start()

    def login(self):
        """
        Called by DCSS.connect() after DCSS connection established.
        """
        super(RobotDHS, self).login()
        self.robot.setup()
        self.send_set_status_string()
        self.send_set_state_string()
        self.send_set_robot_cassette_string()
        self.send_set_robot_force_string('left')
        self.send_set_robot_force_string('middle')
        self.send_set_robot_force_string('right')
        self.send_calibration_timestamps()

    def operation_callback(self, operation, handle, stage, message=None, error=None):
        self.log.info('operation_callback: %r %r %r %r %r',
                      operation, handle, stage, message, error)
        if stage == 'end':
            if error:
                operation.operation_error(error)
            else:
                operation.operation_completed(message or 'OK')

    # ***************************************************************
    # ******************** DHS attributes ***************************
    # ***************************************************************

    @property
    def foreground_operation(self):
        # TODO: Should this be handled with a server attribute?
        return self._foreground_operation

    @foreground_operation.setter
    def foreground_operation(self, value):
        if self._foreground_operation != value:
            self._foreground_operation = value
            self.send_set_status_string()

    @property
    def needs_clear(self):
        # TODO: temporary hack
        return self._needs_clear

    @property
    def needs_reset(self):
        # TODO: temporary hack
        return False

    @property
    def needs_calibration(self):
        # TODO: temporary hack
        return False

    @property
    def warning(self):
        # TODO: temporary hack
        return ''

    @property
    def mounted(self):
        # TODO: Test
        sample_on_goniometer = self.robot.sample_locations['goniometer']
        if not sample_on_goniometer:
            return ''
        return self.port_tuple_to_str(sample_on_goniometer)

    @property
    def sample_state(self):
        # TODO: temporary hack
        return 'on gonio'

    @property
    def dumbbell_state(self):
        # TODO: temporary hack
        return 'in cradle'

    @property
    def manual_mode(self):
        # TODO: temporary hack
        return False

    @property
    def needs_toolset_calibration(self):
        # TODO: temporary hack
        return False

    @property
    def needs_cassette_calibration(self):
        # TODO: temporary hack
        return False

    @property
    def ln2(self):
        if self.robot.ln2_level == 0:
            return 'no'
        elif self.robot.ln2_level == 1:
            return 'yes'
        else:
            return 'wrong'

    @property
    def current_port(self):
        # TODO: temporary hack
        return ''

    @property
    def status(self):
        status = 0
        if self.needs_clear:
            status |= self.STATUS_NEED_CLEAR
        if self.robot.safety_gate:
            status |= self.STATUS_REASON_SAFEGUARD
        if self.robot.at_home != 1:
            status |= self.STATUS_REASON_NOT_HOME
        return status

    @property
    def state(self):
        if self.foreground_operation:
            return self.foreground_operation.name
        return 'idle'

    # ****************************************************************
    # ******************** EPICS callbacks ***************************
    # ****************************************************************

    def on_task_message(self, value):
        self.send_set_status_string()
        try:
            level, message = value.split(' ', 1)
        except ValueError:
            self.log.error('Expected space in %r', value)
            level, message = 'INFO', value
        level = {
            'DEBUG': 'note',
            'INFO': 'note',
            'WARNING': 'warning',
            'ERROR': 'error',
        }.get(level, 'error')
        self.send_xos3('htos_log %s %s' % (level, message))

    def on_system_error_message(self, value):
        if value != 'OK':
            self.send_xos3('htos_log error %s' % value)

    def on_port_distances(self, value):
        # TODO: Need to know which position
        self.send_set_robot_force_string('left')
        self.send_set_robot_force_string('middle')
        self.send_set_robot_force_string('right')


    def on_at_home(self, _): self.send_set_status_string()

    def on_lid_command(self, _): self.send_set_output_string()

    def on_gripper_command(self, _): self.send_set_output_string()

    def on_lid_open(self, _): self.send_set_input_string()

    def on_lid_closed(self, _): self.send_set_input_string()

    def on_gripper_open(self, _): self.send_set_input_string()

    def on_gripper_closed(self, _): self.send_set_input_string()

    def on_pins_mounted(self, _): self.send_set_status_string()

    def on_pins_lost(self, _): self.send_set_status_string()

    def on_task_progress(self, _): self.send_set_status_string()

    def on_closest_point(self, _): self.send_set_state_string()

    def on_ln2_level(self, _): self.send_set_state_string()

    def on_port_states(self, _): self.send_set_robot_cassette_string()

    def on_holder_types(self, _): self.send_set_robot_cassette_string()

    def on_sample_locations(self, _):
        self.send_set_state_string()
        self.send_set_robot_cassette_string()

    # ****************************************************************
    # ******************** dhs -> dcss messages **********************
    # ****************************************************************

    def send_set_output_string(self):
        msg = (
            'htos_set_string_completed robot_output normal '
            '0 '  # out0
            '{robot.gripper_command} '
            '0 '  # out2
            '{robot.lid_command} '
            '0 0 0 0 0 0 0 0 0 '  # out4-14
            '0 '  # TODO: dry air
            '0 '  # TODO: heater
            '0'  # out15
        ).format(robot=self.robot)
        self.send_xos3(msg)

    def send_set_input_string(self):
        msg = (
            'htos_set_string_completed robot_input normal '
            '0 0 0 0 0 0 0 0 '  # in0-7
            '{robot.gripper_open} '
            '{robot.gripper_closed} '
            '0 '  # in10
            '{robot.lid_closed} '
            '{robot.lid_open} '
            '0 '  # TODO: Heater
            '0 0'  # in14-15
        ).format(robot=self.robot)
        self.send_xos3(msg)

    def send_set_status_string(self):
        """
        "status:" status_num         0-400000000
        "need_reset:" need_reset         0 or 1
        "need_cal:" need_calibration   0 or 1
        "state:" state              {idle} {prepare_mount_crystal}
        "warning:" warning message    {empty port in mounting}
        "cal_msg:" calibration message {touching seat} {....}
        "cal_step:" calibration steps  {d of d} {+d} {-d}
        "mounted:" {} or port position been mounted like {l 4 A}
        "pin_lost:" number of pin lost
        "pin_mounted:" number of pin mounted from last reset
        """
        msg = ('htos_set_string_completed robot_status '
               'normal '  # Always "normal"
               'status: {0.status:d} '
               'need_reset: {0.needs_reset:d} '
               'need_cal: {0.needs_calibration:d} '
               'state: {{{0.state}}} '
               'warning: {{{0.warning}}} '
               'cal_msg: {{{robot.task_message}}} '
               'cal_step: {{{robot.task_progress}}} '  # TODO: should validate
               'mounted: {{{0.mounted}}} '
               'pin_lost: {0.robot.pins_lost} '
               'pin_mounted: {0.robot.pins_mounted} '
               'manual_mode: {0.manual_mode:d} '
               'need_mag_cal: {0.needs_toolset_calibration:d} '
               'need_cas_cal: {0.needs_cassette_calibration:d} '
               'need_clear: {0.needs_clear:d}').format(self, robot=self.robot)
        self.send_xos3(msg)

    def send_set_state_string(self):
        """
        htos_set_string_completed robot_state normal
            {on gonio} {in cradle}
            P18 no {m 2 A} 110 1 1 1 0 0
            {invalid} {invalid} {invalid}
            27 27 0 0
        """
        # sample_state: no / on tong / on placer / on picker / on goni / bad state
        # dumbbell_state: out / raised / in cradle / in tong / bad state
        # current_point: P0 / P1 / ... / Wrong
        # ln2: no / yes / wrong
        # current_port: m 2 A / invalid
        # pins_mounted pins_lost pins_mounted_before_lost
        # sample_on_goni: 1, 0
        # pins_stripped pins_stripped_short_trip
        # tong_port: m 2 A / invalid
        # picker_port: m 2 A / invalid
        # placer_port: m 2 A / invalid
        # num_puck_pin_mounted num_puck_pin_mounted_short_trip
        # num_pin_moved num_puck_pin_moved

        sample_is_on_goni = bool(self.robot.sample_locations['goniometer'])
        tong_port = self.port_tuple_to_str(self.robot.sample_locations['cavity'])
        picker_port = self.port_tuple_to_str(self.robot.sample_locations['picker'])
        placer_port = self.port_tuple_to_str(self.robot.sample_locations['placer'])

        msg = (
           'htos_set_string_completed robot_state normal '
           '{{{0.sample_state}}} '
           '{{{0.dumbbell_state}}} '
           'P{robot.closest_point} '
           '{0.ln2} '
           '{{{0.current_port}}} '
           '0 0 0 '
           '{sample_is_on_goni:d} '
           '0 0 '
           '{{{tong_port}}} '
           '{{{picker_port}}} '
           '{{{placer_port}}} '
           '0 0 '
           '0 0'
        ).format(self, robot=self.robot, sample_is_on_goni=sample_is_on_goni,
                 tong_port=tong_port, picker_port=picker_port,
                 placer_port=placer_port)
        self.send_xos3(msg)

    def send_set_robot_cassette_string(self):
        # TODO: Test mounted position
        sample_on_goni = self.robot.sample_locations['goniometer']
        mounted_position, mounted_port = (sample_on_goni
                                          if sample_on_goni else (None, None))
        msg = 'htos_set_string_completed robot_cassette normal'
        for position in ['left', 'middle', 'right']:
            states = [PORT_STATE_LOOKUP[s]
                      for s in self.robot.port_states[position]]
            if mounted_position == position:
                states[mounted_port] = 'm'
            msg += ' {type} {states}'.format(
                type=self.robot.holder_types[position],
                states=' '.join(states)
            )
        self.send_xos3(msg)

    def send_calibration_timestamps(self):
        timestamps = [
            self.robot.last_toolset_calibration,
            self.robot.last_left_calibration,
            self.robot.last_middle_calibration,
            self.robot.last_right_calibration,
            self.robot.last_goniometer_calibration,
        ]
        timestamp_strings = []
        for ts in timestamps:
            timestamp_strings.append('{%s}' % ts if ts else '{}')
        msg = ('htos_set_string_completed ts_robot_cal normal ' +
               ' '.join(timestamp_strings))
        self.send_xos3(msg)

    def send_set_robot_force_string(self, position):
        distance_strings = []
        for distance in self.robot.port_distances[position]:
            if distance is None:
                distance_strings.append('uuuu')
            else:
                distance_strings.append('{:.1f}'.format(distance))
        msg = (
            'htos_set_string_completed robot_force_{position} normal'
            ' {height_error} {distances}'
        ).format(
            position=position,
            height_error=self.robot.height_errors[position] or 0,
            distances=' '.join(distance_strings)
        )
        self.send_xos3(msg)

    # ****************************************************************
    # ******************** dcss -> dhs messages **********************
    # ****************************************************************

    def stoh_register_string(self, *args, **kwargs):
        pass

    def stoh_register_operation(self, *args, **kwargs):
        pass

    def stoh_abort_all(self, operation, *args):
        """ Called by BluIce Abort button """
        # TODO: Do anything?
        pass

    def robot_config(self, operation, task, *args):
        func = getattr(self, 'robot_config_' + task, None)
        if func:
            func(operation, *args)
        else:
            self.log.info('Operation robot_config %s is not handled' % task)

    def robot_config_clear(self, operation):
        self._needs_clear = False  # TODO: Actually check!
        self.send_set_status_string()
        operation.operation_completed('OK')

    def robot_config_clear_all(self, operation):
        self._needs_clear = False  # TODO: Actually check!
        self.send_set_status_string()
        operation.operation_completed('OK')

    def robot_config_hw_output_switch(self, operation, output):
        output = int(output)
        if output == self.OUTPUT_GRIPPER:
            func = self.robot.set_gripper
            value = 1 - self.robot.gripper_command
        elif output == self.OUTPUT_LID:
            func = self.robot.set_lid
            value = 1 - self.robot.lid_command
        else:
            return operation.operation_error('Not implemented')
        func(value, callback=partial(self.operation_callback, operation))

    def robot_config_reset_cassette(self, operation):
        """ Called by the "reset all to unknown" BluIce button """
        # TODO: Set holders
        ports = {'left': [1] * 96, 'middle': [1] * 96, 'right': [1] * 96}
        callback = partial(self.operation_callback, operation)
        self.robot.reset_ports(ports, callback=callback)

    def robot_config_set_index_state(self, operation, start, port_count, state):
        """
        Called by right-clicking ports in BluIce

        Examples:
            left cassette column A 1-8 to bad: start='1', port_count='8', state='b'
            middle puck A port 1 to bad: start='98', port_count='1', state='b'
            middle puck B port 1 to bad: start='114', port_count='1', state='b'
        """
        start, port_count = int(start), int(port_count)
        samples_and_type_per_position = SAMPLES_PER_POSITION + 1
        position_index = start // samples_and_type_per_position
        position = ['left', 'middle', 'right'][position_index]
        start = start % samples_and_type_per_position
        # TODO: Set holder bad if start == 0
        start -= 1
        end = start + port_count
        ports = {position: [0] * 96 for position in ['left', 'middle', 'right']}
        ports[position][start:end] = [1] * port_count
        callback = partial(self.operation_callback, operation)
        self.robot.reset_ports(ports, callback=callback)

    def robot_config_set_port_state(self, operation, port, state):
        """ Called by the reset cassette status to unknown button in BluIce """
        return operation.operation_error('Not implemented')
        # TODO: Use new api
        if port.endswith('X0') and state == 'u':
            position = {'l': 'left', 'm': 'middle', 'r': 'right'}.get(port[0])
            self.robot.set_holder_type(position, HOLDER_TYPE_UNKNOWN)
            self.robot.set_port_states(position,
                                       list(range(SAMPLES_PER_POSITION)),
                                       PORT_STATE_UNKNOWN)

    def robot_config_reset_mounted_counter(self, operation):
        self.robot.run_operation('reset_mount_counters')

    def robot_config_probe(self, operation, *ports):
        ports = [int(p) for p in ports]
        n = SAMPLES_PER_POSITION + 1
        spec = {
            'left': ports[1:n],  # Skip the holder type element
            'middle': ports[n+1:2*n],
            'right': ports[2*n+1:3*n],
        }
        self.robot.probe(spec, callback=partial(self.operation_callback, operation))

    def robot_calibrate(self, operation, target, *run_args):
        run_args = ' '.join(run_args)
        if target == 'magnet_post':
            target = 'toolset'
        self.robot.calibrate(target=target, run_args=run_args,
                             callback=partial(self.operation_callback, operation))

    def prepare_mount_crystal(self, operation, *args):
        # Also used by prepare_dismount_crystal and prepare_mount_next_crystal
        self.log.info('prepare_mount_crystal: %r', args)
        # TODO: Check args
        operation.operation_update('OK to prepare')
        callback = partial(self.operation_callback, operation)
        self.robot.prepare_for_mount(callback=callback)

    def prepare_dismount_crystal(self, operation, *args):
        self.log.info('prepare_dismount_crystal: %r', args)
        self.prepare_mount_crystal(operation)

    def prepare_mount_next_crystal(self, operation, *args):
        self.log.info('prepare_mount_next_crystal: %r', args)
        self.prepare_mount_crystal(operation)

    def mount_crystal(self, operation, cassette, row, column, *args):
        # Also used by mount_next_crystal
        self.log.info('mount_crystal: %r %r %r', cassette, row, column)
        cassette = {'r': 'right', 'm': 'middle', 'l': 'left'}[cassette]
        callback = partial(self.operation_callback, operation)
        self.robot.mount(cassette, column, int(row), callback=callback)

    def dismount_crystal(self, operation, cassette, row, column, *_):
        self.log.info('dismount_crystal: %r %r %r', cassette, row, column)
        cassette = {'r': 'right', 'm': 'middle', 'l': 'left'}[cassette]
        callback = partial(self.operation_callback, operation)
        self.robot.dismount(cassette, column, int(row), callback=callback)

    def mount_next_crystal(self, operation,
                           current_cassette, current_row, current_column,
                           cassette, row, column, *args):
        self.log.info('mount_next_crystal %r %r %r %r %r %r',
                      current_cassette, current_row, current_column,
                      cassette, row, column)
        self.mount_crystal(operation, cassette, row, column)

    def robot_standby(self, operation, *args):
        # TODO: Does anything need to be done here?
        finalize_operation(operation, {})

    def port_tuple_to_str(self, port_tuple):
        if not port_tuple:
            return 'invalid'
        position, port = port_tuple
        holder_type = self.robot.holder_types[position]
        if holder_type in {HOLDER_TYPE_CASSETTE, HOLDER_TYPE_CALIBRATION_CASSETTE}:
            column = chr(port // 8 + ord('A'))
            row = port % 8 + 1
            return '{position} {row} {column}'.format(position=position[0],
                                                      column=column, row=row)
        elif holder_type == HOLDER_TYPE_ADAPTOR:
            puck = chr(port // 16 + ord('A'))
            row = port % 16 + 1
            return '{position} {row} {puck}'.format(position=position[0],
                                                    puck=puck, row=row)
        else:
            return 'invalid'


def finalize_operation(operation, result):
    error = result.get('error')
    if error:
        operation.operation_error(error)
    else:
        operation.operation_completed('OK')