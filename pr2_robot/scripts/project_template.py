#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

	# ====== TODO: Convert ROS msg to PCL data ======
	cloud = ros_to_pcl(pcl_msg)

	# ====== TODO: Statistical Outlier Filtering ======
	# create statistical outlier filter
	outlier_filter = cloud.make_statistical_outlier_filter()
	# set the number of neighbours
	outlier_filter.set_mean_k(16)
	# set the std of filtering (mean + 0.3std)
	outlier_filter.set_std_dev_mul_thresh(0.3)
	# filt the point cloud
	cloud = outlier_filter.filter()


	# ====== TODO: Voxel Grid Downsampling ======
	# create the Voxel Grid filter
	vox = cloud.make_voxel_grid_filter()
	# define the leaf size of the vox filter
	LEAF_SIZE = 0.005
	# set the leaf size of voxel filter
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	# vox filt
	cloud_filtered = vox.filter()


	# ====== TODO: PassThrough Filter ======

	# pass through filter in z direction
	# define the passthrough filter
	passthrough = cloud_filtered.make_passthrough_filter()
	# set the filtering axis
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	# set the range of filter
	axis_min = 0.6
	axis_max = 1.3
	passthrough.set_filter_limits(axis_min, axis_max)
	# filt
	cloud_filtered = passthrough.filter()

	# pass through filter in y direction
	# define the passthrough filter
	passthrough = cloud_filtered.make_passthrough_filter()
	# set the filtering axis
	filter_axis = 'y'
	passthrough.set_filter_field_name(filter_axis)
	# set the range of filter
	axis_min = - 0.5
	axis_max =   0.5
	passthrough.set_filter_limits(axis_min, axis_max)
	# filt
	cloud_filtered = passthrough.filter()

	# pass through filter in x direction
	# define the passthrough filter
	passthrough = cloud_filtered.make_passthrough_filter()
	# set the filtering axis
	filter_axis = 'x'
	passthrough.set_filter_field_name(filter_axis)
	# set the range of filter
	axis_min = 0.3
	axis_max = 1.0
	passthrough.set_filter_limits(axis_min, axis_max)
	# filt
	cloud_filtered = passthrough.filter()


	# ====== TODO: RANSAC Plane Segmentation ======
	# create segmentation filter
	seg = cloud_filtered.make_segmenter()
	# set the RANSAC model
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	# set the max distance
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)
	# create the inliers and coeff
	inliers, coefficients = seg.segment()


	# ====== TODO: Extract inliers and outliers ======
	cloud_table = cloud_filtered.extract(inliers, negative=False)
	cloud_objects = cloud_filtered.extract(inliers, negative=True)


	# ====== TODO: Euclidean Clustering ======
	# seperate the color information
	white_cloud = XYZRGB_to_XYZ(cloud_objects)
	tree = white_cloud.make_kdtree()
	# create a clustering object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# set the parameters of DBSCAN
	ec.set_ClusterTolerance(0.015)
	ec.set_MinClusterSize(20)
	ec.set_MaxClusterSize(4000)
	# search the k-d tree
	ec.set_SearchMethod(tree)
	# extract indices for each clusters
	cluster_indices = ec.Extract()


	# ====== TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately ======
	cluster_color = get_color_list(len(cluster_indices))
    	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
		for i, indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0],
											white_cloud[indice][1],
											white_cloud[indice][2],
											rgb_to_float(cluster_color[j])])


	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)


	# ====== TODO: Convert PCL data to ROS messages ======
	ros_cloud_objects = pcl_to_ros(cloud_objects)
	ros_cloud_table = pcl_to_ros(cloud_table)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)


	# ====== TODO: Publish ROS messages ======
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)


# Exercise-3 TODOs:

	# Classify the clusters! (loop through each detected cluster one at a time)
	detected_objects_labels = []
	detected_objects = []


	for index, pts_list in enumerate(cluster_indices):
		# Grab the points for the cluster
		pcl_cluster = cloud_objects.extract(pts_list)

        # ==== TODO: convert the cluster from pcl to ROS using helper function ====
		ros_cluster = pcl_to_ros(pcl_cluster)

		# Extract histogram features
		chists = compute_color_histograms(ros_cluster, using_hsv=True)
		normals = get_normals(ros_cluster)
		nhists = compute_normal_histograms(normals)
		feature = np.concatenate((chists, nhists))

		# Make the prediction, retrieve the label for the result
		# and add it to detected_objects_labels list
		prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)

		# Publish a label into RViz
		label_pos = list(white_cloud[pts_list[0]])
		label_pos[2] += .4
		object_markers_pub.publish(make_label(label,label_pos, index))

		# Add the detected object to the list of detected objects.
		do = DetectedObject()
		do.label = label
		do.cloud = ros_cluster
		detected_objects.append(do)
		print encoder.inverse_transform(prediction)


	# Publish the list of detected objects
	detected_objects_pub.publish(detected_objects)

	# Suggested location for where to invoke your pr2_mover() function within pcl_callback()
	# Could add some logic to determine whether or not your object detections are robust
	# before calling pr2_mover()
	try:
		# print 'yes'
		pr2_mover(detected_objects)
	except rospy.ROSInterruptException:
		pass

# function to load parameters and request PickPlace service
# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: init list to save objs
    dict_list = []
    centroids = [] # to be list of tuples (x, y, z)

    # TODO: get list of objs form parameter server
    objects_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: create the dictionary of dropbox
    dropbox = {}
    for p in dropbox_param:
        dropbox[p['name']] = p['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for obj in objects_param:


        # TODO: get the name of the obj
        object_name = String()
        object_name.data = obj['name']
        print('======')
        print(object_name.data)
        input()

        #set default value of pick_pose
        pick_pose = Pose()
        pick_pose.position.x = 0
        pick_pose.position.y = 0
        pick_pose.position.z = 0

        #set orientation to 0
        pick_pose.orientation.x = 0
        pick_pose.orientation.y = 0
        pick_pose.orientation.z = 0
        pick_pose.orientation.w = 0

        #set place pose orientation to 0
        place_pose = Pose()
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0

        for detected_object in object_list:

            if detected_object.label == object_name.data:

                print(detected_object.label)

                # TODO: Create 'place_pose' for the object
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroids.append(np.mean(points_arr, axis=0)[:3])
                pick_pose_np = np.mean(points_arr, axis=0)[:3]

                pick_pose.position.x = np.asscalar(pick_pose_np[0])
                pick_pose.position.y = np.asscalar(pick_pose_np[1])
                pick_pose.position.z = np.asscalar(pick_pose_np[2])

                print("------")
                print("centroid " + object_name.data + " is :")
                print(np.mean(points_arr, axis=0)[:3])
                break


        # TODO: Assign the arm to be used for pick_place
        arm_name = String()
        if obj['group'] == 'red':
            arm_name.data = 'left'
        elif obj['group'] == 'green':
            arm_name.data = 'right'
        else:
            print "ERROR, group must be green or red!"


        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        test_scene_num = Int32()
        test_scene_num.data = 3

        place_pose.position.x = dropbox[arm_name.data][0]
        place_pose.position.y = dropbox[arm_name.data][1]
        place_pose.position.z = dropbox[arm_name.data][2]
        dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

    # TODO: Output your request parameters into output yaml file
    yaml_filename = "output_" + str(test_scene_num.data) + ".yaml"

    send_to_yaml(yaml_filename, dict_list)



if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('object_recognition', anonymous=True)

	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

	# TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

	object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

	# TODO: Load Model From disk
	model = pickle.load(open('model.sav', 'rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']

	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
