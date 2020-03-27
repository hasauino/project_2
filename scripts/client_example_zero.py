#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from project_2.msg import MoveBaseAction, MoveBaseGoal


rospy.init_node("sinusoidal_path_node")



msg = MoveBaseGoal()
msg.x = [0]
msg.y = [0]

cl = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

cl.wait_for_server()

cl.send_goal(msg)