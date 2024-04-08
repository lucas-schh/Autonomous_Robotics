#!/usr/bin/env python2
# ROS specific imports
#import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)



try:
    tc.Constant(duration=5.0,linear=-0.2,angular=0.0)
    tc.Wait(duration=1.0)
    tc.Constant(duration=8.0,linear=0.0,angular=1.5)
    tc.Wait(duration=1.0)
    tc.StartExploration(task_timeout=1800.0)

except TaskException as e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.Wait(duration=1.0)
    tc.Constant(duration=2.0,linear=0.0,angular=0.0)
    tc.GoTo(goal_x=-1,goal_y=0, max_velocity=0.5)
    tc.AutoDock()


rospy.loginfo("Mission completed")
