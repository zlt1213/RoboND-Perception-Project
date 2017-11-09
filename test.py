Skip to content
This repository
Search
Pull requests
Issues
Marketplace
Explore
 @zlt1213
 Sign out
 Watch 1
  Star 0  Fork 0 S2H-Mobile/RoboND-Perception-Project
 Code  Issues 0  Pull requests 0  Projects 0  Wiki  Insights
Branch: master Find file Copy pathRoboND-Perception-Project/scripts/object_recognition.py
fcf3edc  on 22 Aug
@S2H-Mobile S2H-Mobile refactor: Streamline print statements
1 contributor
RawBlameHistory
395 lines (298 sloc)  13.8 KB
#!/usr/bin/env python

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


def pcl_callback(pcl_msg):
    """ROS callback function for the Point Cloud Subscriber. Takes a point cloud and
       performs object recognition on it. The pipeline is documented in the writeup.
       Publishes point clouds containing recognized objects and clustered objects.
       Uses machine learning model to classify objects in the cluster.
       Publishes the labeled object list to ROS.
       Calls the pr2_mover routine with the resulting list of detected objects.
    """

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(8)

    # Any point with a mean distance larger than global
    # (mean_distance + 0.3 *std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(0.3)
    cloud = outlier_filter.filter()

    # Voxel Grid filter
    vox = cloud.make_voxel_grid_filter()

    # Define leaf size
    LEAF_SIZE = 0.005

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # PassThrough filter 0.6 < z < 1.1
    pz = cloud_filtered.make_passthrough_filter()
    pz.set_filter_field_name('z')
    pz.set_filter_limits(0.6, 1.1)
    cloud_filtered = pz.filter()

    # PassThrough filter 0.34 < x < 1.0
    px = cloud_filtered.make_passthrough_filter()
    px.set_filter_field_name('x')
    px.set_filter_limits(0.34, 1.0)
    cloud_filtered = px.filter()

    # RANSAC plane segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(40)
    ec.set_MaxClusterSize(4000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, idx in enumerate(indices):
            x = white_cloud[idx][0]
            y = white_cloud[idx][1]
            z = white_cloud[idx][2]
            c = rgb_to_float(cluster_color[j])
            color_cluster_point_list.append([x, y, z, c])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS
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

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Handle the resulting list of objects
    if detected_objects:

        # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)

        # Call the pr2_mover routine
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.loginfo("No objects detected.")


def pr2_mover(detected_objects):
    """ Connect to ROS service for pick and place routine.
        Create request parameters as ROS messages.
        Save request parameters as yaml files.
    """

    # Initialize object list
    objects = rospy.get_param('/object_list')

    # Check consistency of detected objects list
    if not len(detected_objects) == len(objects):
        rospy.loginfo("List of detected objects does not match pick list.")
        return

    # Assign number of objects
    num_objects = len(objects)

    # Initialize test scene number
    num_scene = rospy.get_param('/test_scene_num')

    # Initialize message for test scene number
    test_scene_num = Int32()
    test_scene_num.data = num_scene

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Initialize dropbox positions from ROS parameter
    dropbox = rospy.get_param('/dropbox')
    red_dropbox_position = dropbox[0]['position']
    green_dropbox_position = dropbox[1]['position']

    # Initialize counter for evaluating the accuracy of the prediction
    hit_count = 0

    # Create list of ground truth labels
    true_labels = [element['name'] for element in objects]

    # For each detected object, compare the predicted label with the
    # ground truth from the pick list.
    for detected_object in detected_objects:

        # Initialize predicted label
        predicted_label = detected_object.label

        # compare prediction with ground truth
        if predicted_label in true_labels:

            # remove detected label from ground truth
            true_labels.remove(predicted_label)

            # count successful prediction
            hit_count += 1
        else:

            # mark unsuccessful detection
            detected_object.label = 'error'

    # Log the accuracy
    rospy.loginfo('Detected {} objects out of {}.'.format(hit_count, num_objects))

    # Create list of detected objects sorted in the order of the pick list
    sorted_objects = []

    # Iterate over the pick list
    for i in range(num_objects):

        # Grab the label of the pick list item
        pl_item_label = objects[i]['name']

        # Find detected object corresponding to pick list item
        for detected_object in detected_objects:
            if detected_object.label == pl_item_label:

                 # Append detected object to sorted_objects list
                sorted_objects.append(detected_object)

                # Remove current object
                detected_objects.remove(detected_object)
                break

    # Create lists for centroids and dropbox groups
    centroids = []
    dropbox_groups = []
    for sorted_object in sorted_objects:

        # Calculate the centroid
        pts = ros_to_pcl(sorted_object.cloud).to_array()
        centroid = np.mean(pts, axis=0)[:3]

        # Append centroid as <numpy.float64> data type
        centroids.append(centroid)

        # Search for the matching dropbox group
        # Assuming 1:1 correspondence between sorted objects and pick list
        for pl_item in objects:

            # Compare objects by their label
            if pl_item['name'] == sorted_object.label:

                # Matching object found, add the group to the list
                dropbox_groups.append(pl_item['group'])
                break

    # Initialize list of request parameters for later output to yaml format
    request_params = []

    # Iterate over detected objects to generate ROS message for each object
    for j in range(len(sorted_objects)):

            # Create 'object_name' message with label as native string type
            object_name = String()
            object_name.data = str(sorted_objects[j].label)

            # Initialize the dropbox group
            object_group = dropbox_groups[j]

            # Create 'arm_name' message
            arm_name = String()

            # Select right arm for green group and left arm for red group
            arm_name.data = 'right' if object_group == 'green' else 'left'

            # Convert <numpy.float64> data type to native float as expected by ROS
            np_centroid = centroids[j]
            scalar_centroid = [np.asscalar(element) for element in np_centroid]

            # Create 'pick_pose' message with centroid as the position data
            pick_pose = Pose()
            pick_pose.position.x = scalar_centroid[0]
            pick_pose.position.y = scalar_centroid[1]
            pick_pose.position.z = scalar_centroid[2]

            # Create 'place_pose' message with dropbox center as position data
            place_pose = Pose()
            dropbox_position = green_dropbox_position if object_group == 'green' else red_dropbox_position
            place_pose.position.x = dropbox_position[0]
            place_pose.position.y = dropbox_position[1]
            place_pose.position.z = dropbox_position[2]

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            request_params.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            # Call 'pick_place_routine' service
            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
                response = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
                print("Response: {}".format(response.success))
            except rospy.ServiceException, e:
                print("Service call failed: {}".format(e))

    # Write request parameters to output yaml file
    file_name = "output_{}.yaml".format(num_scene)
    send_to_yaml(file_name, request_params)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('object_detection', anonymous=True)

    # Create subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

© 2017 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
API
Training
Shop
Blog
About
