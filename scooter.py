import rospy

import Jetson.GPIO
import serial
import smbus

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


class Scooter(object):
    def __init__(self):
        rospy.init_node('scooter')

        # Configure UART
        port = rospy.get_param('~serial_port', '/dev/ttyS0')
        self.uart = serial.Serial(port, 115200, timeout=1)

        self.r = rospy.Rate(10)

        self.velocity_set = 0

        self.velocity_last = None

    def _connect(self):

        print("Connecting...")

        while not rospy.is_shutdown():
            startup = "\xAA\x02\x01\x01\xFD"
            self.uart.write(startup)

            recv = serial.read(256)

            if recv[0] == "\xAA" and recv[6] == "\xAC":
                rospy.info("Connected!")
                break

            self.r.sleep()

    @staticmethod
    def crc_int(data):
        sum = 0
        for c in data:
            sum += data
        return sum % 256

    @staticmethod
    def crc_chr(data):
        sum = 0
        for c in data:
            sum += chr(data)
        return sum % 256

    def tx(self, velocity):

        def get_digit(number, n):
            return number // 10**n % 10

        data = []

        # Carriage Return (Message Start)
        data.extend(0x0D)

        # Unknown (maybe sign of velocity?)
        data.extend([0])

        # Velocity
        data.extend([get_digit(velocity, 2) + 0x30,
                     get_digit(velocity, 1) + 0x30,
                     get_digit(velocity, 0) + 0x30])

        # TODO: Unknown
        data.extend([0x30, 0x30, 0x31, 0x31,
                     0x33, 0x31, 0x30, 0x30,
                     0x30, 0x31, 0x30, 0x30,
                     0x31, 0x31, 0x30, 0x31,
                     0x30, 0, 0, 0, 0, 0, 3])

        # CRC
        transmit.append(Scooter.int_crc(transmit))

        # Newline (Message End)
        transmit.append(0x0A)

        # Convert to string
        message = "".join([chr(d) for d in data])

        self.uart.write(message)

    def rx(self):

        message = serial.read(256)
        if message[0] != "\x0D" or message[-1] != "\x0A":
            rospy.warn("Received corrupted packet from scooter.")
            return

        crc_received = chr(message[-2])
        crc_computed = Scooter.crc_chr(message[:-1])

        if crc_received != crc_computed:
            rospy.warn("RX CRC Error.")
            return

        # Startup Message
        if message[1] == "\xA1":
            pass

        # Startup Message?
        elif message[1] == "\x30":
            pass

        # Normal message?
        elif message[1] == "\x31":

            # TODO: positive/negative velocity?
            velocity = int(message[4]) * 100 + int(message[5]) * 10 + int(message[6]) * 1

            if self.velocity_last is not None:
                

            self.velocity_last = velocity


            pass

    def run(self):

        self._connect()

        while not rospy.is_shutdown():

            self.tx(self.velocity_set)
            self.rx()

            self.r.sleep()
