#!/usr/bin/env python3
import serial, select, time, math

SERIAL_TIMEOUT = 0.1
ADDRESSES = ('A', 'B')
#DT = 0.5
POLL_DELAY = 0.01
CHAR_DELAY = 0.01
COMMAND_DELAY = 0.1
MIN_DEVICE_ROTATE_SPEED = 0     # device units; 0.5 deg/s per unit
MAX_DEVICE_ROTATE_SPEED = 80    # device units; max ~40 deg/s

## \brief Class for controlling a PT25 device.
#
# This class provides methods to control a PT25 device over a serial connection.
class pt25:
    ## \brief Initializes the PT25 object.
    #  \param device The serial port device name.
    #  \param baudrate The baud rate for the serial connection.
    def __init__(self, device, baudrate):
        self.port = device
        self.baudrate = baudrate
        self.serial_timeout = SERIAL_TIMEOUT
        self.settings = dict()

        self.init_serial()
        self.last_command = 0.
        self.user_max_rotate_speed = MAX_DEVICE_ROTATE_SPEED

    ## \brief Initializes the serial connection.
    def init_serial(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE, timeout=self.serial_timeout,
                           xonxoff=0, rtscts=0)
            self.ser.nonblocking()
        except serial.serialutil.SerialException as msg:
            print('Failed to open %s (%d): %s' % (self.port, self.baudrate, str(msg)))
            exit()

    ## \brief Sets the counterclockwise limit for the specified address.
    #  \param address The address of the device.
    #  \param limit The limit value to set.
    def set_ccw_limit(self, address, limit):
        if address in ADDRESSES:
            self.send(address+'d'+str(int(limit)).zfill(3))
            time.sleep(POLL_DELAY)
            self.read()
            print('Set CCW limit for %s: %d' % (address, limit))
            return 0
        else:
            print('Invalid address: %s' % address)

    ## \brief Sets the clockwise limit for the specified address.
    #  \param address The address of the device.
    #  \param limit The limit value to set.
    def set_cw_limit(self, address, limit):
        if address in ADDRESSES:
            self.send(address+'u'+str(int(limit)).zfill(3))
            time.sleep(POLL_DELAY)
            self.read()
            print('Set CW limit for %s: %d' % (address, limit))
            return 0
        else:
            print('Invalid address: %s' % address)

    ## \brief Stops the device at the specified address.
    #  \param address The address of the device.
    def stop(self, address):
        time.sleep(POLL_DELAY*4)
        self.send(address + 's128')
        time.sleep(POLL_DELAY*4)
        self.read()                 # <<< consume echo / ack
        time.sleep(POLL_DELAY*4)

    ## \brief Sets the position of the device at the specified address.
    #  \param address The address of the device.
    #  \param position The position to set.
    def set(self, address, position):
        if address in ADDRESSES:
            # Formula only valid from 1 to 359.5.
            if position >= 1.0 and position <= 359.5:
                counts = math.ceil(position / (360./float(self.settings[address]['user_cw_limit'] - self.settings[address]['user_ccw_limit'])) + float(self.settings[address]['user_ccw_limit']) + 0.5)
            # Some special cases
            elif position >= 0 and position < 0.5:
                counts = self.settings[address]['user_ccw_limit']
            elif position >= 0.5 and position < 1.0:
                counts = self.settings[address]['user_ccw_limit'] + 1
            elif position > 359.5:
                counts = self.settings[address]['user_cw_limit']
            # Out of bounds
            else:
                print('Position out of bounds: %.3f' % position)
                return -1
            print('Moving to position %.3f (counts: %d)' % (position, counts))
            time.sleep(POLL_DELAY*4)
            self.send(address+'p'+str(int(counts)).zfill(3))
            time.sleep(POLL_DELAY)
            self.read()
            return 0
        else:
            print('Invalid address: %s' % address)
            return -1

    ## \brief Gets the settings for the device at the specified address.
    #  \param address The address of the device.
    def get_settings(self, address):
        if address in ADDRESSES:
            self.send(address+'?000')
            time.sleep(POLL_DELAY)
            data = self.read().strip()
            if data.__len__() < 2:
                print('Response too short: %s' % data)
                return -1
            data_list = data.split(',')
            if data_list.__len__() >= 10:
                self.settings[address] = dict()
                self.settings[address]['factory_ccw_limit'] = int(data_list[1])
                self.settings[address]['factory_cw_limit'] = int(data_list[2])
                self.settings[address]['user_ccw_limit'] = int(data_list[3])
                self.settings[address]['user_cw_limit'] = int(data_list[4])
                self.settings[address]['pcb_dash_number'] = int(data_list[5])
                self.settings[address]['position_feedback_enable'] = (data_list[6] == 'y')
                self.settings[address]['pcb_serial_number'] = int(data_list[7])
                self.settings[address]['baud_rate'] = int(data_list[8])
                self.settings[address]['positioner_type'] = int(data_list[9])
                self.settings[address]['firmware_revision'] = int(data_list[10])
                print('Got settings for address %s.' % address)
                print(self.settings[address])
                return 0
            else:
                print('Failed to get settings for address %s.' % address)
                return -1
        else:
            print('Invalid address: %s' % address)
            return -1

    ## \brief Polls the device at the specified address.
    #  \param address The address of the device.
    def poll(self, address):
        if address in ADDRESSES:
            self.send(address+'f')
            time.sleep(POLL_DELAY)
            data = self.read().strip()

            if data.__len__() < 3:
                print('Response too short: %s' % data)
                return -1
            if data[0:2] != address+'f':
                print('Invalid echo: %s' % data)
                return -2
            if data[2] != address:
                print('Wrong address: %s' % data[2])
                return -3
            try:
                data_int = int(data[3:])
                data_deg = 360. * float(data_int - self.settings[address]['factory_ccw_limit']) / float(self.settings[address]['factory_cw_limit'] - self.settings[address]['factory_ccw_limit'])
                return data_deg
            except:
                print('Failed to parse %s.' % data[3:])
                return -4
        else:
            print('Invalid address: %s.' % address)
            return -5

    ## \brief Sends a command to the device.
    #  \param tx_str The command string to send.
    def send(self, tx_str):
        if time.time() < self.last_command + COMMAND_DELAY:
            time.sleep(max(0., (self.last_command + COMMAND_DELAY) - time.time()))
        self.last_command = time.time()
        for character in tx_str:
            self.ser.write(character.encode('utf-8'))
            time.sleep(CHAR_DELAY)

    ## \brief Reads data from the serial connection.
    #  \return The data read from the serial connection.
    def read(self):
        try:
            data = self.ser.readline()
            data = data.decode('utf-8')
        except:
            data = ''
            print('Failed to read from serial.')
        return data

    ## \brief Polls the device once.
    def spin_once(self):
        rfds, wfds, efds = select.select([self.ser.fileno()], [], [], self.serial_timeout)
        if rfds.__len__() > 0 and rfds[0] == self.ser.fileno():
            data = self.read()
            print(data)
        else:
            pass

    def _zero_pad_speed(self, n: int) -> str:
        return str(int(n)).zfill(3)

    def _sanitize_speed(self, speed: int) -> int:
        """Clamp speed to device legal range and user max."""
        if speed is None:
            speed = self.user_max_rotate_speed
        speed = int(speed)
        speed = max(MIN_DEVICE_ROTATE_SPEED, min(speed, MAX_DEVICE_ROTATE_SPEED))
        speed = min(speed, self.user_max_rotate_speed)
        return speed

    def set_user_max_rotate_speed(self, max_rotate_speed: int):
        """Set the user cap (still bounded by device MAX)."""
        if max_rotate_speed < MIN_DEVICE_ROTATE_SPEED or max_rotate_speed > MAX_DEVICE_ROTATE_SPEED:
            print(f'Invalid user max rotate speed: {max_rotate_speed} (must be {MIN_DEVICE_ROTATE_SPEED}-{MAX_DEVICE_ROTATE_SPEED})')
            return -1
        self.user_max_rotate_speed = int(max_rotate_speed)
        print(f'User max rotate speed set to {self.user_max_rotate_speed}')
        return 0

    def rotate_ccw(self, address, rotate_speed: int = None):
        """Rotate CCW at given speed (0..80)."""
        if address not in ADDRESSES:
            print(f'Invalid address: {address}')
            return -1
        spd = self._sanitize_speed(rotate_speed)
        cmd = address + '<' + self._zero_pad_speed(spd)
        self.send(cmd)
        time.sleep(POLL_DELAY)
        self.read()
        time.sleep(POLL_DELAY)
        return 0

    def rotate_cw(self, address, rotate_speed: int = None):
        """Rotate CW at given speed (0..80)."""
        if address not in ADDRESSES:
            print(f'Invalid address: {address}')
            return -1
        spd = self._sanitize_speed(rotate_speed)
        cmd = address + '>' + self._zero_pad_speed(spd)
        self.send(cmd)
        time.sleep(POLL_DELAY)
        self.read()
        time.sleep(POLL_DELAY)
        return 0

    def rotate(self, address, signed_speed: int):
        """
        Convenience: signed_speed < 0 => CCW, > 0 => CW, 0 => stop().
        Speed magnitude is clamped to [0..user_max..device_max].
        """
        if address not in ADDRESSES:
            print(f'Invalid address: {address}')
            return -1
        signed_speed = int(signed_speed)
        if signed_speed == 0:
            self.stop(address)
            return 0
        spd = abs(self._sanitize_speed(abs(signed_speed)))
        if signed_speed < 0:
            return self.rotate_ccw(address, spd)
        else:
            return self.rotate_cw(address, spd)


if __name__ == '__main__':
    pt25obj = pt25('/dev/ttyUSB0', 9600)
    pt25obj.get_settings('A')
    pt25obj.get_settings('B')
    while True:
        pt25obj.poll('A')
        pt25obj.poll('B')
        time.sleep(1.0)
