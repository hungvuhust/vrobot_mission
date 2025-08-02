from service_cli import *
import pdb
import sys
import time
rclpy.init()
robot = ServiceCli()
pdb.set_trace()
try:
    robot.move_to_pose("mapmoi", 1)
    robot.move_to_pose("mapmoi", 2)

except Exception as e:
    print('_ERROR_')
pdb.set_trace = lambda: None
rclpy.shutdown() 