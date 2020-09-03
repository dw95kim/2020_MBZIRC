#!/usr/bin/env python
## Author: Hanseob, Lee
## Date: May, 15, 2018
#Purpose: Ros node to detect object using tensorflow

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile

import tensorflow as tf

import zipfile
import cv2
import object_detection
from collections import defaultdict
from io import StringIO
import matplotlib
from matplotlib import pyplot as plt

#for ros:
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import tf as trans
import math

#for time
import time

# ## Object detection imports
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

GAZEBO_SIMULATION = True

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.7

### Define Variable ###
flag = []
pos_img = []
size_img = []
pos_pre = []
image_height = 720
image_width  = 960
image_center_x = 480.0
image_center_y = 360.0
FOCAL_LENGTH_X = 337.349908
FOCAL_LENGTH_Y = 336.028551

# # Detection
class detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.detect_pub = rospy.Publisher("/uav1/detection_fire", Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        # self.image_sub = rospy.Subscriber("/scout/forward/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.image_depth, queue_size=1, buff_size=2**24)
        self.local_pos_sub = rospy.Subscriber("/uav1/mavros/global_position/local", Odometry, self.uav1_odom_callback, queue_size=2)
        #self.image_sub = rospy.Subscriber("/qsf/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        #self.image_sub = rospy.Subscriber("/husky/camera/aligned_depth_to_color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        #self.image_sub = rospy.Subscriber("/husky/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.pos_img = [image_center_x, image_center_y]
        self.size_img = [0, 0]
        self.flag = 0
        self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.pos_img[0], self.pos_img[1]]
        self.PATH_TO_CKPT = '/home/usrg-asus/MBZ_ws/src/3_object_detection/data/models/frozen_inference_graph_ptgrey_overfitting.pb'
        self.PATH_TO_LABELS = '/home/usrg-asus/MBZ_ws/src/3_object_detection/data/labels/labelmap_4.pbtxt'
        self.NUM_CLASSES = 1
        self.is_initialized = 0
        self.depth_img = []
        self.target_depth = 0.0
        self.target_center_x = 0.0
        self.target_center_y = 0.0

        self.Cur_Pos_m = [0.0, 0.0, 0.0]
        self.q = []
        self.euler = [0.0, 0.0, 0.0]

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        #config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.sess = tf.Session(graph=self.detection_graph, config=config)
        # ## Loading label map
        # Label maps map indices to category names, so that when our convolution
        # network predicts `5`, we know that this corresponds to `airplane`.  Here
        # we use internal utility functions, but anything that returns a
        # dictionary mapping integers to appropriate string labels would be fine
        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

    def uav1_odom_callback(self, data):

        self.Cur_Pos_m[0] = data.pose.pose.position.x
        self.Cur_Pos_m[1] = data.pose.pose.position.y
        self.Cur_Pos_m[2] = data.pose.pose.position.z

        self.q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)

    def image_depth(self, data):
        objArray = Detection2DArray()
        if self.is_initialized == 0:
            return 1
        try:
            if not GAZEBO_SIMULATION :
                cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            else :
                cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
           print(e)
        self.depth_img = np.array(cv_image, dtype=np.float32)

        if(self.flag == 1):
            distance = self.depth_img[int(self.pos_img[1])-self.size_img[1]/4:int(self.pos_img[1])+self.size_img[1]/4, int(self.pos_img[0])-self.size_img[0]/4:int(self.pos_img[0])+self.size_img[0]/4]
            if GAZEBO_SIMULATION :
                distance = distance[(distance > 0.2)]
                distance = np.nanmin(distance)            
                self.target_depth = distance
            else :
                distance = distance[(distance > 200)]
                distance = np.nanmin(distance)            
                self.target_depth = distance/1000.0

    def image_cb(self, data):
        print("here")

        global image_height, image_width, image_center_x, image_center_y
        objArray = Detection2DArray()
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

        image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if self.is_initialized == 0 and cv_image.shape[2] == 3:
           image_height, image_width, _ = image.shape
           image_center_x = float(image_width/2.0)
           image_center_y = float(image_height/2.0)

        time_start = time.clock()
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = np.asarray(image)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        with self.detection_graph.as_default():
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})

        objects = vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=2)

        time_elapsed = (time.clock() - time_start)
        #print("time : ",time_elapsed)

        objArray.detections = []
        objArray.header = data.header
        # print len(objects)
        object_count = 1
        self.flag = 0
        # self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.pos_img[0], self.pos_img[1]]
        self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.target_center_x, self.target_center_y, self.target_depth]
        for i in range(len(objects)):
            #print ("Object Number: %d"  %object_count)
            object_count+=1
            objArray.detections.append(self.object_predict(objects[i],data.header,image_np,cv_image))

            if (objects[i][0] == 1):
                self.flag = 1
                dimensions = objects[i][2]
                self.pos_img = [int((dimensions[1] + dimensions[3])*image_width/2), int((dimensions[0] + dimensions[2])*image_height/2)]
                self.size_img = [int((dimensions[3]-dimensions[1] )*image_width), int((dimensions[2]-dimensions[0])*image_height)]
                
        if self.flag == 1:
            self.target_center_x = (self.pos_img[0] - image_center_x)*self.target_depth/FOCAL_LENGTH_X
            self.target_center_y = (self.pos_img[1] - image_center_y)*self.target_depth/FOCAL_LENGTH_Y

            self.target_center_x = self.target_center_x * math.cos(self.euler[0]) - self.target_center_y * math.sin(self.euler[1])
            self.target_center_y = self.target_center_x * math.sin(self.euler[0]) * math.cos(self.euler[1]) + self.target_center_y * math.cos(self.euler[0]) * math.cos(self.euler[1]) - self.target_depth * math.sin(self.euler[1])

            self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.target_center_x, self.target_center_y, self.target_depth]
            image_np = cv2.putText(image_np, "Dist:"+str("{:.2f}".format(self.target_depth)),(int(self.pos_img[0]-self.size_img[0]/2),int(self.pos_img[1]+self.size_img[1]+10)),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            image_np = cv2.putText(image_np, "Target(X,Y) : ("+str("{:.2f}".format(self.target_center_x))+"   "+str("{:.2f}".format(self.target_center_y))+")",(0,int(image_height-20)),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        print(round(1/time_elapsed, 2), "   [Hz]", self.tar_image)
        img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.detect_pub.publish(data=self.tar_image)
        self.object_pub.publish(objArray)
        self.image_pub.publish(image_out)
        self.is_initialized = 1

    def object_predict(self,object_data, header, image_np,image):
        image_height,image_width,channels = image.shape
        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        object_id=object_data[0]
        object_score=object_data[1]
        dimensions=object_data[2]

        obj.header=header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        obj.bbox.size_y = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x = int((dimensions[3]-dimensions[1] )*image_width)
        obj.bbox.center.x = int((dimensions[1] + dimensions [3])*image_width/2)
        obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_height/2)

        return obj

def main(args):
   obj=detector()
   rospy.init_node('detector_node')

   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("ShutDown")

   cv2.destroyAllWindows()

if __name__=='__main__':
  main(sys.argv)
