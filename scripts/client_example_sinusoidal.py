#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from project_2.msg import MoveBaseAction, MoveBaseGoal


rospy.init_node("sinusoidal_path_node")

x = np.linspace(0, 2*np.pi, 30)
y = 3*np.sin(x)

msg = MoveBaseGoal()
msg.x = x
msg.y = y

cl = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

cl.wait_for_server()

cl.send_goal(msg)