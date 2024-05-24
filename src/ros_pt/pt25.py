#!/usr/bin/env python3
import serial, select, time, math

SERIAL_TIMEOUT = 0.1
ADDRESSES = ('A', 'B')
#DT = 0.5
POLL_DELAY = 0.01
CHAR_DELAY = 0.01
COMMAND_DELAY = 0.1

class pt25:
    def __init__(self, device, baudrate):
        self.port = device
        self.baudrate = baudrate
        self.serial_timeout = SERIAL_TIMEOUT
        self.settings = dict()

        self.init_serial()
        self.last_command = 0.

    def init_serial(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE, timeout=self.serial_timeout,
                           xonxoff=0, rtscts=0)
            self.ser.nonblocking()
        except serial.serialutil.SerialException as msg:
            print('Failed to open %s (%d): %s' % (self.port, self.baudrate, msg.message))
            exit()
        #self.ser_symbol_duration = 10./float(self.baudrate)

    def set_ccw_limit(self, address, limit):
        if address in ADDRESSES:
            self.send(address+'d'+str(int(limit)).zfill(3))
            time.sleep(POLL_DELAY)
            self.read()
            print('Set CCW limit for %s: %d' % (address, limit))
            return 0
        else:
            print('Invalid address: %s' % address)

    def set_cw_limit(self, address, limit):
        if address in ADDRESSES:
            self.send(address+'u'+str(int(limit)).zfill(3))
            time.sleep(POLL_DELAY)
            self.read()
            print('Set CW limit for %s: %d' % (address, limit))
            return 0
        else:
            print('Invalid address: %s' % address)

    def stop(self, address):
        time.sleep(POLL_DELAY*4)
        self.send(address + 's128')
        time.sleep(POLL_DELAY*4)
    def set(self, address, position):
        if address in ADDRESSES:
            # Formula only valid from 1 to 359.5.
            if position >= 1.0 and position <= 359.5:
                counts = math.ceil(position / (360./float(self.settings[address]['factory_cw_limit'] - self.settings[address]['factory_ccw_limit'])) + float(self.settings[address]['factory_ccw_limit']) + 0.5)
            # Some special cases
            elif position >= 0 and position < 0.5:
                counts = self.settings[address]['factory_ccw_limit']
            elif position >= 0.5 and position < 1.0:
                counts = self.settings[address]['factory_ccw_limit'] + 1
            elif position > 359.5:
                counts = self.settings[address]['factory_cw_limit']
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


    def poll(self, address):
        if address in ADDRESSES:
            self.send(address+'f')
            time.sleep(POLL_DELAY)
            data = self.read().strip()
            #data = data.decode('utf-8')

            # # Remove 'Af' from the beginning of the data if it exists
            # if data.startswith(b'Af'):
            #     data = data[2:]

            if data.__len__() < 3:
                print('Response too short: %s' % data)
                return -1
            if data[0:2] != address+'f':
                print('Invalid echo: %s' % data)
                return -1
            if data[2] != address:
                print('Wrong address: %s' % data[2])
                return -1
            try:
                data_int = int(data[3:])
                data_deg = 360. * float(data_int - self.settings[address]['factory_ccw_limit']) / float(self.settings[address]['factory_cw_limit'] - self.settings[address]['factory_ccw_limit'])
                #print('Got data: %d  Deg: %.2f  Raw: %s' % (data_int, data_deg, data))
                return data_deg
            except:
                print('Failed to parse %s.' % data[3:])
                return -1
        else:
            print('Invalid address: %s.' % address)
            return -1

    def send(self, tx_str):
        if time.time() < self.last_command + COMMAND_DELAY:
            time.sleep(max(0., (self.last_command + COMMAND_DELAY) - time.time()))
        self.last_command = time.time()
        #print('tx: %s' % tx_str)
        for character in tx_str:
            self.ser.write(character.encode('utf-8'))
            time.sleep(CHAR_DELAY)

    def read(self):
        try:
            data = self.ser.readline()
            data = data.decode('utf-8')
        except:
            data = ''
            print('Failed to read from serial.')
        #print('rx: %s' % data)
        return(data)

    def spin_once(self):
        #if time.time() >= self.next_poll:
            #self.poll('A')
            #self.poll('B')
        #    self.next_poll += self.dt
        #    remaining_time = self.next_poll - time.time()
            #print('Remaining time until next poll: %.3f' % remaining_time)
        #serial_timeout = max(0, self.next_poll - time.time())
        rfds, wfds, efds = select.select([self.ser.fileno()], [], [], self.serial_timeout)
        if rfds.__len__() > 0 and rfds[0] == self.ser.fileno():
            data = self.read()
            print(data)
        else:
            pass
            #print('No traffic on %s (%d).' % (self.port, self.baudrate))

if __name__ == '__main__':
    pt25obj = pt25('/dev/ttyUSB0', 9600)
    pt25obj.get_settings('A')
    pt25obj.get_settings('B')
    #pt25obj.set('A', 90)
    #pt25obj.set('B', 180)
    while True:
        pt25obj.poll('A')
        pt25obj.poll('B')
        time.sleep(1.0)
        #pt25obj.spin_once()
