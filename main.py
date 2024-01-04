import rospy
import argparse

from husky.husky_controller import HuskyControl
from ur5.ur5_controller import UR5Control
from gripper.gripper_controller import GripperControl

from threading import Condition

if __name__ == "__main__":
    rospy.init_node("main controller", anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_keyboard", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    condition = Condition()

    husky_controller = HuskyControl(use_keyboard=args.use_keyboard, debug=args.debug,
                                    condition=condition)
    ur5_controller = UR5Control("arm", use_keyboard=args.use_keyboard, debug=args.debug,
                                condition=condition)
    gripper_controller = GripperControl(use_keyboard=args.use_keyboard, condition=condition)

    rospy.spin()
    
    gripper_controller.close()
    ur5_controller.close()
    husky_controller.close()