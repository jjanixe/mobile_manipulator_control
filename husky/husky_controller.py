import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry

POSSIBLE_KEYS = [
    "w", "s", "q", "e"
]

class HuskyControl(object):
    def __init__(self, use_keyboard=False, debug=False, condition=None):
        '''
        subscribe to keyboard input and publish to husky_velocity_controller/cmd_vel
        '''
        # cmd_vel publisher
        self.velocity_publisher = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=10)
        #self.pose_subscriber = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, self.update_pose)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)
        
        self.use_keyboard = use_keyboard

        self.speed = 0.1 # TODO: remove hardcoding
        self.turn = 0.1 # TODO: remove hardcoding

        if self.use_keyboard:
            self.keyboard_sub = rospy.Subscriber("keyboard_input", String, callback=self.control_with_keyboard)
        else:
            raise NotImplementedError
        
        self.condition = condition

    def callback(self, data):
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    def control_with_keyboard(self, key_input):
        key_input = key_input.data
        key_input = key_input.lower()

        self.condition.acquire()
        if key_input in POSSIBLE_KEYS:
            vel_msg = Twist()
            
            if key_input == "w":
                vel_msg.linear.x = self.speed
            elif key_input == "s":
                vel_msg.linear.x = -self.speed
            
            if key_input == "q":
                vel_msg.angular.z = self.turn
            elif key_input == "e":
                vel_msg.angular.z = -self.turn

            self.velocity_publisher.publish(vel_msg)
        self.condition.notify()
        self.condition.release()

    def close(self):
        pass
