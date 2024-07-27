from arm.robot import Robot
from arm.dynamixel import Dynamixel
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import cv2

class MoveItControl:

    def __init__(self, dry_run=False):
        rospy.init_node('moveit_control')
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback)

        self.dynamic = Dynamixel.Config(
            baudrate=1_000_000,
            device_name='/dev/ttyACM0'
        ).instantiate()
        self.robot = Robot(self.dynamic)

        self.dry_run = dry_run
        if not self.dry_run:
            self.robot._enable_torque()

    def rad2code(self, rad):
        return int(2048 + rad / np.pi * 2048)

    def callback(self, data):
        position = np.array(data.position)
        position = np.array([self.rad2code(pos) for pos in position])
        print(f'position {position}')

        if not self.dry_run:
            self.robot.set_goal_pos(position)




if __name__ == "__main__":
    moveit_control = MoveItControl()

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break