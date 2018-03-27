#!/usr/bin/env python
import sys
import rospy
from affordance_service.srv import *
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import tf

pcb_stamped = PolygonStamped()
hdd_stamped = PolygonStamped()

hdd_stamped.polygon = Polygon()
#p1 = Point32(0, 0, 0)
#p2 = Point32(0, 2, 0)
#p3 = Point32(2, 2, 0)
#p4 = Point32(2, 0, 0)
#hdd_stamped.polygon.points = [p1, p2, p3, p4]


pcb_stamped = PolygonStamped()
#p1 = Point32(0.1, 0.1, 0)
#p2 = Point32(0.1, 1, 0)
#p3 = Point32(1.8, 1, 0)
#p4 = Point32(1.8, 0.1, 0)
#pcb_stamped.polygon.points = [p1, p2, p3, p4]

leverup_poses = PoseArray()


def affordance_client():
    rospy.wait_for_service('asc_service')
    try:
        handle_asc_service = rospy.ServiceProxy('asc_service', ComputeLeverAffordances)
        resp1 = handle_asc_service(hdd_stamped, pcb_stamped)
        #print('Response')
        #print resp1
        return resp1.sum, resp1.affordance_pts
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]


def hdd_callback(msg):
    global hdd_stamped
    hdd_stamped = msg


def pcb_callback(msg):
    global pcb_stamped
    pcb_stamped = msg


def leverup_talker():
    lever_pub = rospy.Publisher('lever_up_points', PoseArray, queue_size=10)
    hdd_sub = rospy.Subscriber('/Shapes/HDD', PolygonStamped, hdd_callback)
    pcb_sub = rospy.Subscriber('/Shapes/PCB', PolygonStamped, pcb_callback)
    tf_listener = tf.TransformListener()

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 10hz

    while not rospy.is_shutdown():
        #print('Sekiller')
        #print(pcb_stamped)
        #print(hdd_stamped)

        if pcb_stamped.polygon.points :

            now = rospy.Time.now()
            status, aff_points = affordance_client()

            lp = []
            for pt in aff_points:

                lp.append(Pose(Point(pt.x,pt.y,pt.z),Quaternion()))

            leverup_poses.poses = lp
            leverup_poses.header.frame_id=pcb_stamped.header.frame_id
            #print(aff_points)
            #rospy.loginfo(aff_points)
            #print(leverup_poses)
            #print(pcb_stamped.header.frame_id)

            lever_pub.publish(leverup_poses)

        rate.sleep()


if __name__ == "__main__":
    try:
        leverup_talker()
    except rospy.ROSInterruptException:
        pass
