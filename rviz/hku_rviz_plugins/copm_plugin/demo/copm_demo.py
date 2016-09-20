#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped

from time import sleep

if __name__ == "__main__":
  rospy.init_node("copm_test")

  cop_pub = rospy.Publisher('copm_plugin_test', Vector3Stamped)

  vec = Vector3Stamped()
  vec.header.frame_id = "map"

  for i in [x/10.0 for x in range(-10, 10, 1)]:
    vec.header.stamp = rospy.Time.now()
    vec.vector.x = i
    cop_pub.publish(vec)
    sleep(1)
    if rospy.is_shutdown():
      break

  
