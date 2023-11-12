#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
# from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import Flaot64
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose

def create_header():
    h = Header()
    h.stamp = rospy.Time.now()
    return h
def aimPx():
    rospy.init_node('aim_px_node')
    pub = rospy.Publisher('aim_px', Detection2DArray, queue_size=100)
    rate = rospy.Rate(10)
    aim_data_x = 100
    aim_data_y = 100
    aim_array_msg = Detection2DArray()
    header = create_header()
    aim_array_msg.header = header

    aim_msg = Detection2D()
    aim_msg.header = header

    source_img = Image()
    aim_msg.source_img = source_img

    result = ObjectHypothesisWithPose()
    aim_msg.results.append(result)

    bbox = BoundingBox2D()

    center = Pose2D()
    center.x = aim_data_x
    center.y = aim_data_y
    center.theta = 1.08
    bbox.center = center

    bbox.size_x = 20
    bbox.size_y = 20

    aim_msg.bbox = bbox

    aim_array_msg.detections.append(aim_msg)
    while not rospy.is_shutdown():
        pub.publish(aim_array_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        aimPx()
    except rospy.ROSInterruptException:
        pass
