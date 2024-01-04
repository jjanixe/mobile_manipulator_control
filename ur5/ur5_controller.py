import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

POSSIBLE_KEYS = [
    "i", "j", "k", "l",
    "u", "o",
    "[", "]",
    "h"
]

class UR5Control(object):
    def __init__(self, group_name="arm", use_keyboard=False, 
                 display=False, debug=False, condition=None,
                 move_dist=0.05, rot_dist=0.01):
        # initialize rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display = display

        self.use_keyboard = use_keyboard
        self.move_dist = move_dist
        self.rot_dist = rot_dist
        
        self.debug = debug
        self.condition = condition
        
         # set robot to face bottom
        self.initialize_robot()
        if self.use_keyboard:
            self.keyboard_sub = rospy.Subscriber("keyboard_input", String, callback=self.control_with_keyboard)
        else:
            raise NotImplementedError

        if self.display:
            self.display_trajectory_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20,
            )

        # We can get the name of the reference frame for this robot:
        if self.debug:
            planning_frame = self.move_group.get_planning_frame()
            print("============ Planning frame: %s" % planning_frame)

            # We can also print the name of the end-effector link for this group:
            eef_link = self.move_group.get_end_effector_link()
            print("============ End effector link: %s" % eef_link)

            # We can get a list of all the groups in the robot:
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", self.robot.get_group_names())

            # Sometimes for debugging it is useful to print the entire state of the
            # robot:
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")

    def initialize_robot(self):
        '''
        initialize ur5 robot to face bottom
        '''
        self.plan_joint_goal([-0.10418445268739873,
                             -0.8313329855548304,
                             -1.9140799681292933,
                             -1.9187944571124476,
                             1.5961856842041016,
                             -0.3330600897418421])
                             
                             
    def plan_joint_goal(self, joint_goal):
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def plan_pose_goal(self, goal):
        self.move_group.set_pose_target(goal)

        success = self.move_group.go(wait=True)
        self.move_group.stop()

        if not success:
            print("EXECUTION FAILED")

        self.move_group.clear_pose_targets()

    def control_with_keyboard(self, key_input):
        '''
        W/A/S/D for husky control
        I/J/K/L / U/O / [/] for ur5 control
        G for gripper control
        ESC for rospy shutdown
        '''
        key_input = key_input.data
        key_input = key_input.lower()

        self.condition.acquire()
        if key_input in POSSIBLE_KEYS:
            current_pose = self.move_group.get_current_pose().pose
            goal_pose = copy.deepcopy(current_pose)
            
            print("Current key input: {}".format(key_input))

            if key_input == "i":
                goal_pose.position.x +=  self.move_dist
            elif key_input == "j":
                goal_pose.position.y += self.move_dist
            elif key_input == "k":
                goal_pose.position.x -= self.move_dist
            elif key_input == "l":
                goal_pose.position.y -= self.move_dist
            
            if key_input == "u":
                goal_pose.position.z += self.move_dist
            elif key_input == "o":
                goal_pose.position.z -= self.move_dist
            
            if key_input == ("["):
                print("rotation not possible yet")
                pass
            elif key_input == ("]"):
                print("rotation not possible yet")
                pass

            if key_input == "h":
                # return to initial pose
                self.initialize_robot()
            
            if self.debug:        
                print(goal_pose)
            
            if key_input is not "h":
                # publish goal pose
                self.plan_pose_goal(goal_pose)
        
        self.condition.notify()
        self.condition.release()

    def close(self):
        # return to initial pose
        self.initialize_robot()

if __name__=="__main__":
    controller = UR5Control("arm", use_keyboard=True)
