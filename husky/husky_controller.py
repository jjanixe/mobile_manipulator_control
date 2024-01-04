import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry

POSSIBLE_KEYS = [
    "q", "Q", "w", "W", "e", "E",
    "a", "A", "s", "S", "d", "D",
    "z", "Z", "x", "X", "c", "C",
]

moveBindings = {
    "q": (1, 1, 0, 0),
    "w": (1, 0, 0, 0),
    "e": (1, -1, 0, 0),
    "a": (0, 1, 0, 0),
    "s": (0, 0, 0, 0),
    "d": (0, -1, 0, 0),
    "z": (-1, 1, 0, 0),
    "x": (-1, 0, 0, 0),
    "c": (-1, -1, 0, 0),
}

class HuskyControl(object):
    def __init__(self, use_keyboard=False, debug=False):
        '''
        subscribe to keyboard input and publish to husky_velocity_controller/cmd_vel
        '''
        # cmd_vel publisher
        self.velocity_publisher = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, self.update_pose)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)

        self.speed = 0.1 # TODO: remove hardcoding
        self.turn = 1.0 # TODO: remove hardcoding

        if self.use_keyboard:
            self.keyboard_sub = rospy.Subscriber("keyboard_input", String, callback=self.control_with_keyboard)
        else:
            raise NotImplementedError
        
        self.lock = False
        self.using = rospy.Publisher("")

    def callback(self, data):
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    def control_with_keyboard(self, key_input):
        key_input = key_input.data

        self.condition.acquire()
        if key_input in POSSIBLE_KEYS:
            key_input = key_input.lower()

            vel_msg = Twist()

            x = moveBindings[key_input][0]
            y = moveBindings[key_input][1]
            z = moveBindings[key_input][2]
            th = moveBindings[key_input][3]

            # TODO: update speed

            # publish to cmd_vel
            vel_msg.linear.x = x*self.speed
            vel_msg.linear.y = y*self.speed
            vel_msg.linear.z = z*self.speed
            vel_msg.angular.z = th*self.turn

            self.velocity_publisher.publish(vel_msg)
        self.condition.notify()
        self.condition.release()

    def close(self):
        pass