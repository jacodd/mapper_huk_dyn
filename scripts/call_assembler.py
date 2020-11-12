#!/usr/bin/env python

import rospy; 
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2

rospy.init_node("assembler")
rospy.wait_for_service("assemble_scans2")

assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher("/assembled_pointcloud", PointCloud2, queue_size=1)

r = rospy.Rate(0.5)

while not rospy.is_shutdown():
    try:
        capture_point_cloud_360 = rospy.get_param("capture_point_cloud_360", True)
        #print "capture_point_cloud_360: %s" % capture_point_cloud_360
        if not capture_point_cloud_360:
            break

        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        #print "Got cloud with %u points" % len(resp.cloud.data)
        pub.publish(resp.cloud)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    

    r.sleep()