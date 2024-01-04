import threading
import rospy
from std_msgs.msg import String

import sys
from select import select

import termios
import tty

msg = """
Reading from the keyboard and Publishing!
---------------------------
Husky control:
    - w / a / s / d : forward, left, backward, right
    - q / e : rotate left, rotate right

UR5 control:
    - i / j / k / l : forward, left, backward, right
    - u / o : move up, move down

Gripper control:
    - g: close / release

CTRL-C to quit
"""

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('keyboard_input', String, queue_size = 1)
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, str_input):
        self.condition.acquire()
        self.str_input = str_input
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update("_")
        self.join()

    def run(self):
        str_msg = String()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            self.condition.release()

            # Publish.
            self.publisher.publish(self.str_input)

        self.publisher.publish("_")


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat)

    str_input = "_"
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(str_input)

        print(msg)
        while(1):
            key = getKey(settings, key_timeout)
            if (status == 14):
                print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and str_input=="_":
                    continue
                str_input = "_"
                if (key == '\x03'):
                    break

            pub_thread.update(key)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)