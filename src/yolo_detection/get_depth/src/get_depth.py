#!/usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D,PoseStamped
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from scipy.spatial.transform import Rotation as R

bridge = CvBridge()

class GetDepth:
    def __init__(self):
        self.puv_refreshed = False
        self.odom_refreshed = False
        self.pu = None
        self.py = None
        self.sub_c = rospy.Subscriber("/yolov7/yolov7", Detection2DArray, self.callback_center, queue_size=1) # for test: /aim_px
        self.sub_d = rospy.Subscriber("iris_D435i/realsense/depth_camera/depth/image_raw", Image, self.callback_depth,
                                      queue_size=1)
        self.sub_odom = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.callback_odom, # /visual_slam/odom or /mavros/local_position/odom
                                      queue_size=1)
        self.pub_depth = rospy.Publisher("obj_depth", Float64, queue_size=5)
        self.pub_position = rospy.Publisher("obj_position", PoseStamped,queue_size=5)
        self.body_pos_x = None
        self.body_pos_y = None
        self.body_pos_z = None
        self.quat_1 = None
        self.quat_2 = None
        self.quat_3 = None
        self.quat_4 = None
        self.obj_x = None
        self.obj_y = None
        self.obj_z = None

    def callback_depth(self,data=Image):
        global depth_img
        try:
            # print('get image')
            # print("in get_depth, before  imgmsg2cv, shape img:", data.height,'x',data.width) # for test
            depth_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")# desired_encoding="passthrough" or "16UC1"
            # print("in get_depth, after imgmsg2cv, shape img:", depth_img.shape) # for test
            # depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
            # print("in get_depth, after cvtcolr,not imgmsg2cv, shape img:", depth_img.shape) # for test
            # depth_img_bgr = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR) # added
            if self.puv_refreshed == True & self.odom_refreshed == True:
                depth_data = float(depth_img[int(self.pu), int(self.pv)])
                self.puv_refreshed = False
                self.odom_refreshed = False
                depth_msg = Float64()
                depth_msg.data = depth_data
                self.pub_depth.publish(depth_msg)
                print('depth_msg_data', depth_msg.data)


                # calculate actual position in world frame
                zd = depth_data
                fx = 376.0
                fy = 376.0
                cx = 376.0
                cy = 240.0

                xm = zd/fx*(self.pu - cx)
                ym = zd/fy*(self.pv - cy)

                # xk = -xm
                # yk = zk
                # zk = ym
                xk = zd
                yk = -xm
                zk = -ym

                xb = xk + 0.12
                yb = yk
                zb = zk + 0.015

                quat_list = [self.quat_1,self.quat_2,self.quat_3,self.quat_4]
                Rm = R.from_quat(quat_list)     
                # print("type_Rm:",type(Rm)) 
                rotation_matrix = Rm.as_matrix()

                o2b_position_list = [xb,yb,zb]
                o2b_position_matrix = np.zeros((3,1))
                for i in range(3):
                    o2b_position_matrix[i] = o2b_position_list[i]

                body_position_list = [self.body_pos_x,self.body_pos_y, self.body_pos_z]
                body_position_matrix = np.zeros((3,1))
                for i in range(3):
                    body_position_matrix[i] = body_position_list[i]

                p =  np.dot(rotation_matrix,o2b_position_matrix) + body_position_matrix
                self.obj_x,self.obj_y,self.obj_z = p[0:3]
                # print('self.obj_xyz:\n',self.obj_x,self.obj_y,self.obj_z,'\n')

                # Generate obj position msg
                obj_position_msg = PoseStamped()
                obj_position_msg.pose.position.x = self.obj_x-0.2 # modified for not collide
                obj_position_msg.pose.position.y = self.obj_y
                obj_position_msg.pose.position.z = self.obj_z #!!!!!
                obj_position_msg.pose.orientation.x = 0
                obj_position_msg.pose.orientation.y = 0
                obj_position_msg.pose.orientation.z = 0
                obj_position_msg.pose.orientation.w = 1

                # publish obj_position
                self.pub_position.publish(obj_position_msg)
                print('body position list:\n',body_position_list,'\n obj postion p:\n',p,'\n')

            # else:
            #     print("callback_depth funtion got no aim point!")
        except CvBridgeError as e:
            print(e)

    def callback_center(self, data=Detection2DArray):
        if len(data.detections) !=0: # modified for prevent error when onthing to detect
            self.pu = data.detections[0].bbox.center.y # modified for image shape confuse
            self.pv = data.detections[0].bbox.center.x
            # print('self.pu ,pv', self.pu, self.pv) # for test
            # print('get aim pixel')
            self.puv_refreshed = True
    
    def callback_odom(self,data = Odometry):
        self.body_pos_x = data.pose.pose.position.x
        self.body_pos_y = data.pose.pose.position.y
        self.body_pos_z = data.pose.pose.position.z
        self.quat_1 = data.pose.pose.orientation.x
        self.quat_2 = data.pose.pose.orientation.y
        self.quat_3 = data.pose.pose.orientation.z
        self.quat_4 = data.pose.pose.orientation.w
        self.odom_refreshed = True
        # print( 'get odom info:')#,self.body_pos_x ,self.body_pos_y ,self.body_pos_z)




rospy.init_node('get_depth_node')
publisher = GetDepth()

while not rospy.is_shutdown():
    rospy.spin()
