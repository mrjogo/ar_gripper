import logging
from threading import Lock

import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)


class ResponseError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return "Servo error: " + repr(self.value)


class CommunicationError(RuntimeError):
    def __init__(self, text):
        RuntimeError.__init__(self, text)


class USB2FeetechDevice:
    """
    Serial connection to Feetech devices on RS485 bus.
    """

    def __init__(self, device, baudrate=115200):
        try:
            self.device = int(
                device
            )  # stores the serial port as 0-based integer for Windows
        except ValueError:
            self.device = (
                device  # stores it as a /dev-mapped string for Linux / Mac
            )

        self.lock = Lock()

        self._servo_dev = None

        with self.lock:
            self._open_serial(baudrate)

    @property
    def servo_dev(self):
        return self._servo_dev

    def write(self, msg):
        """ Caller must acquire/release the mutex"""
        self._servo_dev.write(msg)

    def flush(self):
        self._servo_dev.flushInput()

    def read(self, n_bytes=1):
        """ Caller must acquire/release the mutex"""
        reply = self._servo_dev.read(n_bytes)
        if len(reply) < n_bytes:
            raise CommunicationError(
                f"read_serial: not enough bytes received (expected {n_bytes},"
                f" received {len(reply)})"
            )
        return reply

    def _open_serial(self, baudrate):
        try:
            self._servo_dev = serial.serial_for_url(
                url=self.device,
                baudrate=baudrate,
                timeout=0.2,
                writeTimeout=0.2,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except serial.serialutil.SerialException as e:
            raise RuntimeError("lib_feetech: Serial port not found!\n%s" % e)
        if self._servo_dev is None:
            raise RuntimeError('lib_feetech: Serial port not found!\n')
        self._servo_dev.flushOutput()
        self._servo_dev.flushInput()


class FeetechServo:
    """
    Feetech SMS servo control.
    """

    device = None  # type: USB2FeetechDevice

    def __init__(self, feetech_device, servo_id, retry_count=3):
        self.retry_count = retry_count

        if feetech_device is None:
            raise RuntimeError("Feetech servo requires a feetech USB device")
        else:
            self.device = feetech_device

        self._servo_id = servo_id
        try:
            self._read_address(3)
            retry = False
        except CommunicationError as e:
            if self.retry_count == 0:
                raise
            logger.warning(f"Exception: {e}")
            logger.warning("Get ID failed once, retrying")
            retry = True
        if retry:
            self.device.flush()
            try:
                self._read_address(3)
            except CommunicationError:
                raise RuntimeError(
                    f"lib_feetech: Error encountered. "
                    f"Could not find ID {self._servo_id} "
                    f"on bus {self.device.device}.\n'"
                )

        # Set Return Delay time -
        # Used to determine when next status can be requested
        data = self._read_address(0x05, 1)
        self._return_delay = data[0] * 2e-6

    def _send_instruction(self, instruction, exception_on_error_response=True):
        """
        send instruction with retries
        """
        msg = [self._servo_id, len(instruction) + 1] + instruction
        checksum = self._calc_checksum(msg)
        msg = [0xFF, 0xFF] + msg + [checksum]

        with self.device.lock:
            failures = 0
            while True:
                try:
                    self.device.flush()
                    self._send_serial(msg)
                    data, err = self._receive_reply()
                    if err & 16 != 0:
                        raise CommunicationError(
                            "Servo error 16 - sent packet checksum invalid"
                        )
                    break
                except (CommunicationError, serial.SerialException) as e:
                    failures += 1
                    if failures > self.retry_count:
                        raise
                    logger.warning(
                        f"send_instruction retry {failures}, error: {e}"
                    )
        if not exception_on_error_response:
            return data, err
        if err != 0:
            raise ResponseError(err)
        return data

    def _read_address(self, address, n_bytes=1):
        """
        reads nBytes from address on the servo.
        returns [n1,n2 ...] (list of parameters)
        """
        msg = [0x02, address, n_bytes]
        return self._send_instruction(msg)

    def _write_address(self, address, data):
        """
        writes data at the address.
        data = [n1,n2 ...] list of numbers.
        return [n1,n2 ...] (list of return parameters)
        """
        msg = [0x03, address] + data
        return self._send_instruction(msg)

    def _send_serial(self, msg):
        """
        sends the command to the servo
        """
        out = bytearray(msg)
        self.device.write(bytes(out))

    def _receive_reply(self):
        start_bytes_received = 0
        # skipped_bytes = []
        while start_bytes_received < 2:
            one = self.device.read(1)
            # print ord(one[0])
            if one == b'\xff':
                start_bytes_received += 1
            else:
                # skipped_bytes += one
                start_bytes_received = 0

        # if len(skipped_bytes) > 0:
        #    print 'Skipped bytes:', skipped_bytes
        servo_id = bytearray(self.device.read(1))
        if servo_id[0] != self._servo_id:
            raise CommunicationError(
                f'lib_robotis: Incorrect servo ID received: {servo_id[0]},'
                f' expected {self._servo_id}'
            )
        data_len = bytearray(self.device.read(1))
        err = bytearray(self.device.read(1))
        data = self.device.read(data_len[0] - 2)
        chksum_in = bytearray(self.device.read(1))[0]
        chksum_calc = self._calc_checksum(servo_id + data_len + err + data)
        if chksum_calc != chksum_in:
            raise CommunicationError(
                f'Checksum mismatch: calculated {hex(chksum_calc)},'
                f' received {hex(chksum_in)}'
            )
        return data, err[0]

    @staticmethod
    def _calc_checksum(msg):
        chksum = sum(msg)
        chksum = (~chksum) % 256
        return chksum


class FeetechSMSServo(FeetechServo):
    BAUDRATE_VALUES = [
        1000000,
        500000,
        250000,
        128000,
        115200,
        76800,
        57600,
        38400,
    ]

    _FIRMWARE_MAIN_VERSION_ADDR = 0x00
    _FIRMWARE_SECONDARY_VERSION_ADDR = 0x01
    _SERVO_MAIN_VERSION_ADDR = 0x03
    _SERVO_SECONDARY_VERSION_ADDR = 0x04
    _ID_ADDR = 0x05
    _BAUDRATE_ADDR = 0x06
    _RETURN_DELAY_TIME_ADDR = 0x07
    _STATUS_RETURN_LEVEL_ADDR = 0x08
    _MIN_POSITION_LIMIT_ADDR = 0x09
    _MAX_POSITION_LIMIT_ADDR = 0x0B
    _MAX_TEMPERATURE_LIMIT_ADDR = 0x0D
    _MAX_INPUT_VOLTAGE_ADDR = 0x0E
    _MIN_INPUT_VOLTAGE_ADDR = 0x0F
    _MAX_TORQUE_ADDR = 0x10
    _UNLOADING_CONDITION_ADDR = 0x13
    _LED_ALARM_CONDITION_ADDR = 0x14
    _POS_P_COEFFICIENT_ADDR = 0x15
    _POS_D_COEFFICIENT_ADDR = 0x16
    _POS_I_COEFFICIENT_ADDR = 0x17
    _MINIMUM_STARTUP_FORCE_ADDR = 0x18
    _CW_INSENSITIVE_ZONE_ADDR = 0x1A
    _CCW_INSENSITIVE_ZONE_ADDR = 0x1B
    _PROTECTIVE_CURRENT_ADDR = 0x1C
    _ANGLE_RESOLUTION_ADDR = 0x1E
    _POSITION_CORRECTION_ADDR = 0x1F
    _DRIVE_MODE_ADDR = 0x21
    _PROTECTION_TORQUE_ADDR = 0x22
    _PROTECTION_TIME_ADDR = 0x23
    _OVERLOAD_TORQUE_ADDR = 0x24
    _VEL_P_COEFFICIENT_ADDR = 0x25
    _OVERCURRENT_PROTECTION_TIME_ADDR = 0x26
    _VEL_I_COEFFICIENT_ADDR = 0x27
    _TORQUE_SWITCH_ADDR = 0x28
    _ACCELERATION_ADDR = 0x29
    _GOAL_POSITION_ADDR = 0x2A
    _DRIVE_SPEED_ADDR = 0x2E
    _TORQUE_LIMIT_ADDR = 0x30
    _LOCK_SIGN_ADDR = 0x37
    _PRESENT_POSITION_ADDR = 0x38
    _PRESENT_VELOCITY_ADDR = 0x3A
    _PRESENT_LOAD_ADDR = 0x3C
    _PRESENT_VOLTAGE_ADDR = 0x3E
    _PRESENT_TEMPERATURE_ADDR = 0x3F
    _ASYNCHRONOUS_WRITING_SIGN_ADDR = 0x40
    _MOVING_STATUS_ADDR = 0x41
    _MOVING_SIGN_ADDR = 0x42
    _PRESENT_CURRENT_ADDR = 0x45

    MIN_POS_PROTECTION_BIT = 1 << 0
    MAX_POS_PROTECTION_BIT = 1 << 1
    MAX_TEMP_PROTECTION_BIT = 1 << 2
    MAX_VOLTAGE_PROTECTION_BIT = 1 << 3
    MIN_VOLTAGE_PROTECTION_BIT = 1 << 4
    MAX_TORQUE_PROTECTION_BIT = 1 << 5

    DRIVE_MODE_POSITION_SERVO = 0
    DRIVE_MODE_CONSTANT_SPEED = 1
    DRIVE_MODE_PWM_OPEN_LOOP = 2

    TORQUE_SWITCH_OFF = 0
    TORQUE_SWITCH_ON = 1
    TORQUE_SWITCH_MID = 128  # ?? not sure what the manual means

    def __init__(self, feetech_device, servo_id, retry_count=3):
        super().__init__(feetech_device, servo_id, retry_count)

    @property
    def firmware_version(self):
        data1 = self._read_address(self._FIRMWARE_MAIN_VERSION_ADDR, 1)
        data2 = self._read_address(self._FIRMWARE_SECONDARY_VERSION_ADDR, 1)
        return '{}.{}'.format(data1[0], data2[0])

    @property
    def servo_version(self):
        data1 = self._read_address(self._SERVO_MAIN_VERSION_ADDR, 1)
        data2 = self._read_address(self._SERVO_SECONDARY_VERSION_ADDR, 1)
        return '{}.{}'.format(data1[0], data2[0])

    @property
    def servo_id(self):
        return self._servo_id

    @servo_id.setter
    def servo_id(self, value):
        if not 0 <= value <= 253:
            raise ValueError("Servo ID must be 0-253")
        self._write_address(self._ID_ADDR, [value & 0xFF])
        self._servo_id = value

    @property
    def baudrate(self):
        data = self._read_address(self._BAUDRATE_ADDR, 1)
        return self.BAUDRATE_VALUES[data[0]]

    @baudrate.setter
    def baudrate(self, value):
        if value not in self.BAUDRATE_VALUES:
            raise ValueError(f"Baudrate must be one of {self.BAUDRATE_VALUES}")
        i = self.BAUDRATE_VALUES.index(value)
        self._write_address(self._BAUDRATE_ADDR, [i])

    @property
    def return_delay_time(self):
        data = self._read_address(self._RETURN_DELAY_TIME_ADDR, 1)
        return data[0] * 2

    @return_delay_time.setter
    def return_delay_time(self, value):
        if not 0 <= value <= 508:
            raise ValueError("Return delay time must be between 0 and 508us")
        data = [value // 2]
        self._write_address(self._RETURN_DELAY_TIME_ADDR, data)

    @property
    def status_return_level(self):
        data = self._read_address(self._STATUS_RETURN_LEVEL_ADDR, 1)
        return data[0]

    @status_return_level.setter
    def status_return_level(self, value):
        data = [int(bool(value))]
        self._write_address(self._STATUS_RETURN_LEVEL_ADDR, data)

    @property
    def min_position_limit(self):
        data = self._read_address(self._MIN_POSITION_LIMIT_ADDR, 2)
        return int(data[0] | data[1] << 8)

    @min_position_limit.setter
    def min_position_limit(self, value):
        if not 0 <= value <= 4094:
            raise ValueError(
                "Minimum position limit must be between 0 and 4094"
            )
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._MIN_POSITION_LIMIT_ADDR, data)

    @property
    def max_position_limit(self):
        data = self._read_address(self._MAX_POSITION_LIMIT_ADDR, 2)
        return int(data[0] | data[1] << 8)

    @max_position_limit.setter
    def max_position_limit(self, value):
        if not 1 <= value <= 4095:
            raise ValueError(
                "Maximum position limit must be between 0 and 4094"
            )
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._MAX_POSITION_LIMIT_ADDR, data)

    @property
    def max_temperature_limit(self):
        data = self._read_address(self._MAX_TEMPERATURE_LIMIT_ADDR, 1)
        return data[0]

    @max_temperature_limit.setter
    def max_temperature_limit(self, value):
        if not 0 <= value <= 100:
            raise ValueError(
                "Maximum temperature limit must be between 0 and 100"
            )
        data = [value & 0xFF]
        self._write_address(self._MAX_TEMPERATURE_LIMIT_ADDR, data)

    @property
    def max_input_voltage(self):
        data = self._read_address(self._MAX_INPUT_VOLTAGE_ADDR, 1)
        return data[0] * 0.1

    @max_input_voltage.setter
    def max_input_voltage(self, value):
        if not 0.0 <= value <= 25.4:
            raise ValueError(
                "Maximum input voltage must be between 0 and 25.4V"
            )
        data = [int(value * 10)]
        self._write_address(self._MAX_INPUT_VOLTAGE_ADDR, data)

    @property
    def min_input_voltage(self):
        data = self._read_address(self._MIN_INPUT_VOLTAGE_ADDR, 1)
        return data[0] * 0.1

    @min_input_voltage.setter
    def min_input_voltage(self, value):
        if not 0.0 <= value <= 25.4:
            raise ValueError(
                "Maximum input voltage must be between 0 and 25.4V"
            )
        data = [int(value * 10)]
        self._write_address(self._MIN_INPUT_VOLTAGE_ADDR, data)

    @property
    def max_torque(self):
        data = self._read_address(self._MAX_TORQUE_ADDR, 2)
        return int(data[0] | data[1] << 8) * 0.1

    @max_torque.setter
    def max_torque(self, value):
        if not 0.0 <= value <= 100.0:
            raise ValueError("Maximum torque must be between 0 and 100%")
        value = int(value * 10)
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._MAX_TORQUE_ADDR, data)

    @property
    def unloading_condition(self):
        data = self._read_address(self._UNLOADING_CONDITION_ADDR, 1)
        return data[0]

    @unloading_condition.setter
    def unloading_condition(self, value):
        data = [value & 0xFF]
        self._write_address(self._UNLOADING_CONDITION_ADDR, data)

    @property
    def led_alarm_condition(self):
        data = self._read_address(self._LED_ALARM_CONDITION_ADDR, 1)
        return data[0]

    @led_alarm_condition.setter
    def led_alarm_condition(self, value):
        data = [value & 0xFF]
        self._write_address(self._LED_ALARM_CONDITION_ADDR, data)

    @property
    def position_p_coefficient(self):
        data = self._read_address(self._POS_P_COEFFICIENT_ADDR, 1)
        return data[0]

    @position_p_coefficient.setter
    def position_p_coefficient(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Position P coefficient must be between 0 and 254")
        self._write_address(self._POS_P_COEFFICIENT_ADDR, [value])

    @property
    def position_d_coefficient(self):
        data = self._read_address(self._POS_D_COEFFICIENT_ADDR, 1)
        return data[0]

    @position_d_coefficient.setter
    def position_d_coefficient(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Position D coefficient must be between 0 and 254")
        self._write_address(self._POS_D_COEFFICIENT_ADDR, [value])

    @property
    def position_i_coefficient(self):
        data = self._read_address(self._POS_I_COEFFICIENT_ADDR, 1)
        return data[0]

    @position_i_coefficient.setter
    def position_i_coefficient(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Position P coefficient must be between 0 and 254")
        self._write_address(self._POS_I_COEFFICIENT_ADDR, [value])

    @property
    def minimum_startup_force(self):
        data = self._read_address(self._MINIMUM_STARTUP_FORCE_ADDR, 2)
        return int(data[0] | data[1] << 8) * 0.1

    @minimum_startup_force.setter
    def minimum_startup_force(self, value):
        if not 0.0 <= value <= 100.0:
            raise ValueError("Minimum startup force must be between 0 and 100%")
        value = int(value * 10)
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._MINIMUM_STARTUP_FORCE_ADDR, data)

    @property
    def cw_insensitive_zone(self):
        data = self._read_address(self._CW_INSENSITIVE_ZONE_ADDR, 1)
        return data[0]

    @cw_insensitive_zone.setter
    def cw_insensitive_zone(self, value):
        if not 0 <= value <= 32:
            raise ValueError("CW insensitive zone must be between 0 and 32")
        self._write_address(self._CW_INSENSITIVE_ZONE_ADDR, [value])

    @property
    def ccw_insensitive_zone(self):
        data = self._read_address(self._CCW_INSENSITIVE_ZONE_ADDR, 1)
        return data[0]

    @ccw_insensitive_zone.setter
    def ccw_insensitive_zone(self, value):
        if not 0 <= value <= 32:
            raise ValueError("CCW insensitive zone must be between 0 and 32")
        self._write_address(self._CCW_INSENSITIVE_ZONE_ADDR, [value])

    @property
    def protective_current(self):
        data = self._read_address(self._PROTECTIVE_CURRENT_ADDR, 2)
        return int(data[0] | data[1] << 8) * 6.5

    @protective_current.setter
    def protective_current(self, value):
        if not 0 <= value <= 3250:
            raise ValueError("Protective current must be between 0 and 3250mA")
        value = value // 6.5
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._PROTECTIVE_CURRENT_ADDR, data)

    @property
    def angle_resolution(self):
        data = self._read_address(self._ANGLE_RESOLUTION_ADDR, 1)
        return data[0]

    @angle_resolution.setter
    def angle_resolution(self, value):
        if not 1 <= value <= 100:
            raise ValueError("Angle Resolution must be between 1 and 100")
        self._write_address(self._ANGLE_RESOLUTION_ADDR, [int(value)])

    @property
    def position_correction(self):
        data = self._read_address(self._POSITION_CORRECTION_ADDR, 2)
        return int(data[0] | (data[1] & 0x7) << 8) * (
            -1 if data[1] & 0x8 else 1
        )

    @position_correction.setter
    def position_correction(self, value):
        if not -2047 <= value <= 2047:
            raise ValueError(
                "Position correction must be between -2047 and 2047"
            )
        value = int(value)
        sign = 1 if value < -1 else 0
        data = [value & 0xFF, value & 0x7 | sign << 3]
        self._write_address(self._POSITION_CORRECTION_ADDR, data)

    @property
    def drive_mode(self):
        data = self._read_address(self._DRIVE_MODE_ADDR, 1)
        return data[0]

    @drive_mode.setter
    def drive_mode(self, value):
        if value not in (0, 1, 2):
            raise ValueError("Drive mode must be 0, 1 or 2")
        self._write_address(self._DRIVE_MODE_ADDR, [value])

    @property
    def protection_torque(self):
        data = self._read_address(self._PROTECTION_TORQUE_ADDR, 1)
        return data[0]

    @protection_torque.setter
    def protection_torque(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Protection torque must be between 0 and 254%")
        self._write_address(self._PROTECTION_TORQUE_ADDR, [int(value)])

    @property
    def protection_time(self):
        data = self._read_address(self._PROTECTION_TIME_ADDR, 1)
        return data[0] * 10

    @protection_time.setter
    def protection_time(self, value):
        if not 0 <= value <= 2540:
            raise ValueError("Protection time must be between 0 and 2540ms")
        self._write_address(self._PROTECTION_TIME_ADDR, [value // 10])

    @property
    def overload_torque(self):
        data = self._read_address(self._OVERLOAD_TORQUE_ADDR, 1)
        return data[0]

    @overload_torque.setter
    def overload_torque(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Overload torque must be between 0 and 254%")
        self._write_address(self._OVERLOAD_TORQUE_ADDR, [int(value)])

    @property
    def velocity_p_coefficient(self):
        data = self._read_address(self._VEL_P_COEFFICIENT_ADDR, 1)
        return data[0]

    @velocity_p_coefficient.setter
    def velocity_p_coefficient(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Velocity P coefficient must be between 0 and 254")
        self._write_address(self._VEL_P_COEFFICIENT_ADDR, [value])

    @property
    def overcurrent_protection_time(self):
        data = self._read_address(self._OVERCURRENT_PROTECTION_TIME_ADDR, 1)
        return data[0] * 10

    @overcurrent_protection_time.setter
    def overcurrent_protection_time(self, value):
        if not 0 <= value <= 2540:
            raise ValueError(
                "Overcurrent protection time must be between 0 and 2540ms"
            )
        self._write_address(
            self._OVERCURRENT_PROTECTION_TIME_ADDR, [value // 10]
        )

    @property
    def velocity_i_coefficient(self):
        data = self._read_address(self._VEL_I_COEFFICIENT_ADDR, 1)
        return data[0]

    @velocity_i_coefficient.setter
    def velocity_i_coefficient(self, value):
        if not 0 <= value <= 254:
            raise ValueError("Velocity I coefficient must be between 0 and 254")
        self._write_address(self._VEL_I_COEFFICIENT_ADDR, [value])

    @property
    def torque_enable(self):
        data = self._read_address(self._TORQUE_SWITCH_ADDR, 1)
        return bool(data[0])

    @torque_enable.setter
    def torque_enable(self, value):
        self._write_address(self._TORQUE_SWITCH_ADDR, [int(bool(value))])

    @property
    def acceleration(self):
        data = self._read_address(self._ACCELERATION_ADDR, 1)
        return data[0] * 100

    @acceleration.setter
    def acceleration(self, value):
        if not 0 <= value <= 25400:
            raise ValueError(
                "Acceleration must be between 0 and 25400 steps/s2."
            )
        self._write_address(self._ACCELERATION_ADDR, [value // 100])

    @property
    def goal_position(self):
        data = self._read_address(self._GOAL_POSITION_ADDR, 2)
        return int(data[0] | (data[1] & 0x7F) << 8) * (
            -1 if data[1] & 0x80 else 1
        )

    @goal_position.setter
    def goal_position(self, value):
        if not -32766 <= value <= 32766:
            raise ValueError(
                "Goal position must be between -32766 and 32766 steps."
            )
        value = int(value)
        sign = 1 if value < -1 else 0
        data = [value & 0xFF, value >> 8 & 0x7F | sign << 7]
        self._write_address(self._GOAL_POSITION_ADDR, data)

    @property
    def drive_speed(self):
        data = self._read_address(self._DRIVE_SPEED_ADDR, 2)
        return int(data[0] | data[1] << 8) * 50

    @drive_speed.setter
    def drive_speed(self, value):
        if not 0 <= value <= 122500:
            raise ValueError("Drive speed must be between 0 and 122500 steps/s")
        value = value // 50
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(
            self._DRIVE_SPEED_ADDR,
            data,
        )

    @property
    def torque_limit(self):
        data = self._read_address(self._TORQUE_LIMIT_ADDR, 2)
        return int(data[0] | data[1] << 8) * 0.1

    @torque_limit.setter
    def torque_limit(self, value):
        if not 0.0 <= value <= 100.0:
            raise ValueError("Torque limit must be between 0 and 100%")
        value = int(value * 10)
        data = [value & 0xFF, value >> 8 & 0xFF]
        self._write_address(self._TORQUE_LIMIT_ADDR, data)

    @property
    def write_lock(self):
        data = self._read_address(self._LOCK_SIGN_ADDR, 1)
        return data[0]

    @write_lock.setter
    def write_lock(self, value):
        if value not in (0, 1):
            raise ValueError("Write lock must be 0 or 1.")
        self._write_address(self._LOCK_SIGN_ADDR, [int(value)])

    @property
    def present_position(self):
        data = self._read_address(self._PRESENT_POSITION_ADDR, 2)
        return int(data[0] | (data[1] & 0x7F) << 8) * (
            -1 if data[1] & 0x80 else 1
        )

    @property
    def present_velocity(self):
        data = self._read_address(self._PRESENT_VELOCITY_ADDR, 2)
        return int(data[0] | data[1] << 8) * 50

    @property
    def present_load(self):
        data = self._read_address(self._PRESENT_LOAD_ADDR, 2)
        return int(data[0] | data[1] << 8) * 0.1

    @property
    def present_voltage(self):
        data = self._read_address(self._PRESENT_VELOCITY_ADDR, 1)
        return data[0] * 10

    @property
    def present_temperature(self):
        """
        returns the temperature (Celcius)
        """
        data = self._read_address(self._PRESENT_TEMPERATURE_ADDR, 1)
        return data[0]

    @property
    def asynchronous_writing_sign(self):
        data = self._read_address(self._ASYNCHRONOUS_WRITING_SIGN_ADDR, 1)
        return data[0]

    @property
    def moving_status(self):
        data = self._read_address(self._MOVING_STATUS_ADDR, 1)
        return data[0]

    @property
    def moving_sign(self):
        data = self._read_address(self._MOVING_SIGN_ADDR, 1)
        return data[0]

    @property
    def present_current(self):
        data = self._read_address(self._PRESENT_CURRENT_ADDR, 2)
        return int(data[0] | data[1] << 8) * 6.5

    def reset_current_position(self):
        """
        Sets current position to 2048.
        """
        self._write_address(self._TORQUE_SWITCH_ADDR, [128])

    def flush_all(self):
        self.device.flush()

    def check_overload_and_recover(self):
        pass  # we don't need it yet


def find_servos(device, max_id=252, print_progress=False):
    """ Finds all servo IDs on the USB2Dynamixel """
    servos = []
    prev_timeout = device.servo_dev.timeout
    device.servo_dev.timeout = 0.03  # To make the scan faster
    for i in range(max_id + 1):  # 0..max_id
        try:
            _ = FeetechSMSServo(device, i, retry_count=0)
            if print_progress:
                print("FOUND A SERVO @ ID %d" % i)
            servos.append(i)
        except CommunicationError:
            pass
    device.servo_dev.timeout = prev_timeout
    return servos


def find_servos_on_all_ports(max_id=252, print_progress=False):
    ports = serial.tools.list_ports.comports()
    result = []
    for port in ports:
        device_name = port[0]
        if (
            'ttyUSB' in device_name
            or 'ttyACM' in device_name
            or 'COM' in device_name
        ):
            if print_progress:
                print(f"device: {device_name}")
            try:
                connection = USB2FeetechDevice(device_name)
                servo_ids = find_servos(
                    connection, max_id=max_id, print_progress=print_progress
                )
                if servo_ids:
                    result.append((device_name, servo_ids))
            except RuntimeError as e:
                logger.warning(e)
    return result


if __name__ == '__main__':
    pass
