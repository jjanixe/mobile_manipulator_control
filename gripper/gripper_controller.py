import serial
import time
import binascii

import rospy
from std_msgs.msg import String

POSSIBLE_KEYS = [
    'g', 'G'
]

CLOSE = 0
RELEASE = 1

class GripperControl(object):
    def __init__(self, port='/dev/ttyUSB0',baudrate=115200,timeout=1,
                 use_keyboard=False, debug=False, condition=None):
        self.serial = serial.Serial(port=port,baudrate=baudrate,timeout=timeout,
                                    parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS)
        self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data_raw = self.serial.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("Response 1 ", data)
        time.sleep(0.01)

        self.serial.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
        data_raw = self.serial.readline()
        print(data_raw)

        data = binascii.hexlify(data_raw)
        print ("Response 2 ", data)

        time.sleep(1)

        self.gripper_state = RELEASE
        self.use_keyboard = use_keyboard
        self.debug = debug
        self.condition = condition

        if self.use_keyboard:
            self.keyboard_sub = rospy.Subscriber("keyboard_input", String, callback=self.control_with_keyboard)
        else:
            raise NotImplementedError

    def close_gripper(self):
        self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        data_raw = self.serial.readline()
        if self.debug:
            print(data_raw)
        time.sleep(2)
        self.gripper_state = CLOSE

    def open_gripper(self):
        self.serial.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        data_raw = self.serial.readline()
        if self.debug:
            print(data_raw)
        time.sleep(2)
        self.gripper_state = RELEASE

    def control_with_keyboard(self, key_input):
        self.condition.acquire()
        key_input = key_input.data

        print("Current key input: {}".format(key_input))

        if key_input in POSSIBLE_KEYS:
            if self.gripper_state == CLOSE:
                self.open_gripper()
            else:
                self.close_gripper()
        self.condition.notify()
        self.condition.release()

    def close(self):
        # open gripper before exiting
        self.open_gripper()
