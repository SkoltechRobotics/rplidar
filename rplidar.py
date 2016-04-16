'''Simple and lightweight module for working with RPLidar rangefinder scanners.

Usage example:

>>> from rplidar import RPLidar
>>> lidar = RPLidar('/dev/ttyUSB0')
>>> info =lidar.get_info()
>>> print('\n'.join('%s: %s' % (k, str(v)) for k, v in info.items()))
firmware: (1, 15)
model: 0
hardware: 0
serialnumber: 64E699F3C7E59AF0A2E69DF8F13735
>>> lidar.get_health()
('Good', 0)
>>> process_scan = lambda scan: None
>>> for scan in lidar.iter_scans():
...  process_scan(scan)
KeyboardInterrupt
>>> lidar.stop()
>>> lidar.stop_motor()

For additional information please refer to the RPLidar class documentation.
'''
import logging
import sys
import time
import codecs
import serial

SYNC_BYTE = b'\xA5'
SYNC_BYTE2 = b'\x5A'

GET_INFO_BYTE = b'\x50'
GET_HEALTH_BYTE = b'\x52'

STOP_BYTE = b'\x40'
RESET_BYTE = b'\x40'

SCAN_BYTE = b'\x20'
FORCE_SCAN_BYTE = b'\x21'

DESCRIPTOR_LEN = 7
INFO_LEN = 20
HEALTH_LEN = 3

INFO_TYPE = 4
HEALTH_TYPE = 6
SCAN_TYPE = 129

_HEALTH_STATUSES = {
    0: 'Good',
    1: 'Warning',
    2: 'Error',
}


class RPLidarException(Exception):
    '''Basic exception class for RPLidar'''


def _b2i(byte):
    '''Converts byte to integer (for Python 2 compatability)'''
    return byte if int(sys.version[0]) == 3 else ord(byte)

def _process_scan(raw):
    '''Processes input raw data and returns measurment data'''
    new_scan = bool(_b2i(raw[0]) & 0b1)
    inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
    quality = _b2i(raw[0]) >> 2
    if new_scan == inversed_new_scan:
        raise RPLidarException('New scan flags mismatch')
    check_bit = _b2i(raw[1]) & 0b1
    if check_bit != 1:
        raise RPLidarException('Check bit not equal to 1')
    angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
    distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
    return new_scan, quality, angle, distance


class RPLidar(object):
    '''Class for communicating with RPLidar rangefinder scanners'''

    _serial_port = None  #: serial port connection
    port = ''  #: Serial port name, e.g. /dev/ttyUSB0
    timeout = 1  #: Serial port timeout
    motor = False  #: Is motor running?
    baudrate = 115200  #: Baudrate for serial port

    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        '''Initilize RPLidar object for communicating with the sensor.

        Parameters
        ----------
        port : str
            Serial port name to which sensor is connected
        baudrate : int, optional
            Baudrate for serial connection (the default is 115200)
        timeout : float, optional
            Serial port connection timeout in seconds (the default is 1)
        logger : logging.Logger instance, optional
            Logger instance, if none is provided new instance is created
        '''
        self._serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_running = None
        if logger is None:
            logger = logging.getLogger('rplidar')
        self.logger = logger
        self.connect()
        self.start_motor()

    def connect(self):
        '''Connects to the serial port with the name `self.port`. If it was
        connected to another serial port disconnects from it first.'''
        if self._serial_port is not None:
            self.disconnect()
        try:
            self._serial_port = serial.Serial(
                self.port, self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout)
        except serial.SerialException as err:
            raise RPLidarException('Failed to connect to the sensor '
                                   'due to: %s' % err)

    def disconnect(self):
        '''Disconnects from the serial port'''
        if self._serial_port is None:
            return
        self._serial_port.close()


    def start_motor(self):
        '''Starts sensor motor'''
        self.logger.info('Starting motor')
        self._serial_port.setDTR(False)
        self.motor_running = True

    def stop_motor(self):
        '''Stops sensor motor'''
        self.logger.info('Stoping motor')
        self._serial_port.setDTR(True)
        self.motor_running = False

    def _send_cmd(self, cmd):
        '''Sends `cmd` command to the sensor'''
        req = SYNC_BYTE + cmd
        self._serial_port.write(req)
        self.logger.debug('Command sent: %s' % req)

    def _read_descriptor(self):
        '''Reads descriptor packet'''
        descriptor = self._serial_port.read(DESCRIPTOR_LEN)
        self.logger.debug('Recieved descriptor: %s', descriptor)
        if len(descriptor) != DESCRIPTOR_LEN:
            raise RPLidarException('Descriptor length mismatch')
        elif not descriptor.startswith(SYNC_BYTE + SYNC_BYTE2):
            raise RPLidarException('Incorrect descriptor starting bytes')
        is_single = _b2i(descriptor[-2]) == 0
        return _b2i(descriptor[2]), is_single, _b2i(descriptor[-1])

    def _read_response(self, dsize):
        '''Reads response packet with length of `dsize` bytes'''
        self.logger.debug('Trying to read response: %d bytes', dsize)
        data = self._serial_port.read(dsize)
        self.logger.debug('Recieved data: %s', data)
        if len(data) != dsize:
            raise RPLidarException('Wrong body size')
        return data

    def get_info(self):
        '''Get device information

        Returns
        -------
        dict
            Dictionary with the sensor information
        '''
        self._send_cmd(GET_INFO_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != INFO_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != INFO_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        serialnumber = codecs.encode(raw[4:], 'hex').upper()
        serialnumber = codecs.decode(serialnumber, 'ascii')
        data = {
            'model': _b2i(raw[0]),
            'firmware': (_b2i(raw[2]), _b2i(raw[1])),
            'hardware': _b2i(raw[3]),
            'serialnumber': serialnumber,
        }
        return data

    def get_health(self):
        '''Get device health state. When the core system detects some
        potential risk that may cause hardware failure in the future,
        the returned status value will be 'Warning'. But sensor can still work
        as normal. When sensor is in the Protection Stop state, the returned
        status value will be 'Error'. In case of warning or error statuses
        non-zero error code will be returned.

        Returns
        -------
        status : str
            'Good', 'Warning' or 'Error' statuses
        error_code : int
            The related error code that caused a warning/error.
        '''
        self._send_cmd(GET_HEALTH_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != HEALTH_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != HEALTH_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        status = _HEALTH_STATUSES[_b2i(raw[0])]
        error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
        return status, error_code

    def clear_input(self):
        '''Clears input buffer by reading all available data'''
        self._serial_port.read_all()

    def stop(self):
        '''Stops scanning process, disables laser diode and the measurment
        system, moves sensor to the idle state.'''
        self.logger.info('Stoping scanning')
        self._send_cmd(STOP_BYTE)
        time.sleep(.001)
        self.clear_input()

    def reset(self):
        '''Resets sensor core, reverting it to a similar state as it has
        just been powered up.'''
        self.logger.info('Reseting the sensor')
        self._send_cmd(RESET_BYTE)
        time.sleep(.002)

    def iter_measurments(self, max_buf_meas=500, force=False):
        '''Iterate over measurments. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increaing lag.

        Yields
        ------
        new_scan : bool
            True if measurment belongs to a new scan
        quality : int
            Reflected laser pulse strength
        angle : float
            The measurment heading angle in degree unit [0, 360)
        distance : float
            Measured object distance related to the sensor’s rotation center.
            In millimeter unit. Set to 0 when th measurment is invalid.
        '''
        self.start_motor()
        status, error_code = self.get_health()
        self.logger.debug('Health status: %s [%d]', status, error_code)
        if status == _HEALTH_STATUSES[2]:
            self.logger.warning('Trying to reset sensor due to the error. '
                                'Error code: %d', error_code)
            self.reset()
            status, error_code = self.get_health()
            if status == _HEALTH_STATUSES[2]:
                raise RPLidarException('RPLidar hardware failure. '
                                       'Error code: %d' % error_code)
        elif status == _HEALTH_STATUSES[1]:
            self.logger.warning('Warning sensor status detected! '
                                'Error code: %d', error_code)
        cmd = SCAN_BYTE if not force else FORCE_SCAN_BYTE
        self._send_cmd(cmd)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != 5:
            raise RPLidarException('Wrong get_info reply length')
        if is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != SCAN_TYPE:
            raise RPLidarException('Wrong response data type')
        while True:
            raw = self._read_response(dsize)
            self.logger.debug('Recieved scan response: %s' % raw)
            if max_buf_meas:
                data_in_buf = self._serial_port.in_waiting
                if data_in_buf > max_buf_meas*dsize:
                    self.logger.warning(
                        'Too many measurments in the input buffer: %d/%d. '
                        'Clearing buffer...',
                        data_in_buf//dsize, max_buf_meas)
                    self._serial_port.read(data_in_buf//dsize*dsize)
            yield _process_scan(raw)

    def iter_scans(self, max_buf_meas=500, min_len=5, force=False):
        '''Iterate over scans. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increasing lag.

        Yields
        ------
        scan : list
            List of the measurments. Each measurment is tuple with following
            format: (quality, angle, distance). For values description please
            refer to `iter_measurments` method's documentation.
        '''
        scan = []
        for data in self.iter_measurments(max_buf_meas, force):
            if data[0]:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if data[1] > 0:
                scan.append(data[1:])
