#!/usr/bin/env python
## Author: Hanseob, Lee
## Date: May, 15, 2018
#Purpose: Ros node to detect drone using tensorflow

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
from numpy import inf

try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

#for time
import time

# ## Object detection imports
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.7

### Define Variable ###
flag = []
pos_img = []
size_img = []
pos_pre = []
image_height=376
image_width=672
image_center_x = 334.674
image_center_y = 195.668

# # Detection
class detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.detect_pub = rospy.Publisher("/detection", Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/CAM/image_raw", Image, self.image_cb, queue_size=2, buff_size=2**24)
        self.image_sub = rospy.Subscriber("/left/image_raw_color", Image, self.image_cb, queue_size=2, buff_size=2**24)
        #self.depth_sub = rospy.Subscriber("/depth/depth_registered", Image, self.image_depth, queue_size=2, buff_size=2**24)

	self.pos_img = [image_center_x, image_center_y]
	self.size_img = [0, 0]
	self.flag = 0
	self.flag_sinc = 0
	self.flag_depth = 1
	self.dist = 40.0
	self.dist_pre = 40.0
	self.depth_array = []
	self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.pos_img[0], self.pos_img[1], self.dist]

        self.PATH_TO_CKPT = '/home/usrg/dataset/savedModel/frozen_inference_graph.pb'
        # List of the strings that is used to add correct label for each box.
        self.PATH_TO_LABELS = '/home/usrg/dataset/USRG_label_map.pbtxt'
        self.NUM_CLASSES = 3

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
        # dictionary mapping integers to appropriate string labels would be
        # fine
        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

	'''
    def image_depth(self, data):
	try:	     
	   cv_depth = self.bridge.imgmsg_to_cv2(data, "passthrough")	      
	except CvBridgeError as e:
	   print(e)

	self.depth_array=np.array(cv_depth, dtype=np.float32)

	for ind in range(self.size_img[0]):
		for jnd in range(self.size_img[1]):
			self.dist = self.depth_array[self.pos_img[1]-self.size_img[1]/2+jnd][self.pos_img[0]-self.size_img[0]/2+ind]
			if self.dist==inf:	
				self.dist = self.dist_pre
			if self.dist==-inf:	
				self.dist = self.dist_pre
			#print(self.dist)

	self.dist_pre = self.dist

	cv2.normalize(cv_depth, cv_depth, 0, 1, cv2.NORM_MINMAX)
	cv_depth = cv2.rectangle(cv_depth, (self.pos_img[0]-self.size_img[0]/2, self.pos_img[1]-self.size_img[1]/2), (self.pos_img[0]+self.size_img[0]/2, self.pos_img[1]+self.size_img[1]/2), 1, 2)
	cv2.imshow('depth',cv_depth)
	cv2.waitKey(3)
	'''

    def image_cb(self, data):
        objArray = Detection2DArray()

        # tar_image = [flag, size_x, size_y, center_x, center_y]

        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

        image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
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
		    line_thickness=2,
		    max_boxes_to_draw=20)

		time_elapsed = (time.clock() - time_start)
		#print("time : ",time_elapsed)

		objArray.detections = []
		objArray.header = data.header
		# print len(objects)
		object_count = 1
		self.flag = 0
		self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.pos_img[0], self.pos_img[1], self.dist]
		for i in range(len(objects)):
		   #print ("Object Number: %d"  %object_count)
		   object_count+=1
		   objArray.detections.append(self.object_predict(objects[i],data.header,image_np,cv_image))

		   if (objects[i][0] == 1):
		     self.flag = 1
		     dimensions = objects[i][2]
		     self.pos_img = [int((dimensions[1] + dimensions[3])*image_width/2), int((dimensions[0] + dimensions[2])*image_height/2)]
		     self.size_img = [int((dimensions[3]-dimensions[1] )*image_width), int((dimensions[2]-dimensions[0])*image_height)]
		     #tar_image = [flag, size_img[0], size_img[1], pos_img[0], pos_img[1]]
		     #self.tar_image = [self.pos_img[0]-image_center_x, self.pos_img[1]-image_center_y, self.distance_to_obs, 0]
		     self.tar_image = [self.flag, self.size_img[0], self.size_img[1], self.pos_img[0], self.pos_img[1], self.dist]

	 	print(round(1/time_elapsed, 2), "[Hz]", self.tar_image)
		
		self.detect_pub.publish(data=self.tar_image)
		self.object_pub.publish(objArray)
		img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
		image_out = Image()
		try:
		    image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
		except CvBridgeError as e:
		    print(e)
		image_out.header = data.header
		self.image_pub.publish(image_out)
		self.flag_sinc = 1

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
