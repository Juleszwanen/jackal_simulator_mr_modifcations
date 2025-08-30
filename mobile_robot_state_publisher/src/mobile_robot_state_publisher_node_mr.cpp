/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2018 \n
 *   TU Delft
 *
 *****************************************************************
 *
 * \note
 *   Project name:
 * \note
 *   ROS stack name:
 * \note
 *   ROS package name: mobile_robot_state_publisher
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.deBrito@tudelft.nl
 *
 * \date Date of creation: May, 2018
 *
 * \brief
 *   This package provides a generic mobile_robot_stsate_publisher
 *
 ****************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for toMsg/fromMsg conversions
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

geometry_msgs::Pose pos;     // storage of the robots base link pose -> Jules Aug 1: this is now relative to odom frame which is not the same as the map frame
geometry_msgs::Twist vel;    // storage of the robots bae_link vel 
nav_msgs::Odometry odom_msg; //full odometry message that will be published

ros::Publisher state_pub_;
ros::Publisher odom_pub_;

ros::Time last_velocity_callback_;
double rate_;

tf2_ros::Buffer tfBuffer;
// JULES: Added by me
string robot_namespace;
string tf_prefix;
std::string target_link; // e.g., "jackal1::base_link" -> used to extract correct index in gazebo mesage
std::string tf_base_frame;  // TF frame, e.g., "jackal1/base_link"
std::string tf_odom_frame;  // e.g., "jackal1/odom"
std::string tf_child_frame; // for odometry.child_frame_id
// JULES: Stop Added

// The convention of tramsform to publish is the following T_{parent}

void VelocityCallBack(const gazebo_msgs::LinkStates &msg)
{
	// Update at the given rate
	if (last_velocity_callback_ + ros::Duration(1. / rate_) > ros::Time::now())
		return;

	last_velocity_callback_ = ros::Time::now();

	// std::string str2("base_link");

	// We are going to look for the /namespace/base_link link in the /gazebo/link_states topic msg
	bool found_index = false;
	size_t index;
	for (index = 0; index < msg.name.size(); index++)
	{

		if (msg.name[index] == target_link)
		{
			found_index = true;
			break;
		}
	}

	if (!found_index)
	{
		 ROS_WARN_STREAM("Mobile Robot State Publisher: velocity callback() - index not found for "
                        << target_link);
		return;
	}




	// Extract Gazebo ground-truth pose/twist (in /map / world coordinates)
    pos = msg.pose[index];
	
   
    // We'll overwrite odom_msg.pose.pose below after transforming into odom frame

	// Lookup static transform: map → odom and store it in a message which expresses points in the odom frame
	geometry_msgs::TransformStamped odom_from_map_tf_msg;
	try {
		// lookupTransform(target_frame, source_frame, ...)
		// Gives T_target_source
		odom_from_map_tf_msg = tfBuffer.lookupTransform(
								tf_odom_frame,   // target: odom
								"map",           // source: map
								ros::Time(0),    // latest available
								ros::Duration(0.1)); // timeout
	} catch (const tf2::TransformException &ex) {
		ROS_WARN_STREAM("Could not get transform from /map to " 
						<< tf_odom_frame << ": " << ex.what());
		return;
	}

	// Convert to tf2::Transform
	tf2::Transform T_odom_map; // maps points in map → odom frame
	tf2::fromMsg(odom_from_map_tf_msg.transform, T_odom_map);

	// Build T_map_base from Gazebo's ground truth
	tf2::Transform T_map_base; // maps points in base → map frame
	{
		tf2::Vector3 origin(pos.position.x, pos.position.y, pos.position.z);
		tf2::Quaternion base_q(pos.orientation.x, pos.orientation.y,
							pos.orientation.z, pos.orientation.w);
		T_map_base.setOrigin(origin);
		T_map_base.setRotation(base_q);
	}

	// Compose: T_odom_base = T_odom_map * T_map_base
	tf2::Transform T_odom_base = T_odom_map * T_map_base;

	// --- Fill Odometry message with the base expressed in the odometry---
	geometry_msgs::Transform odom_from_base_tf = tf2::toMsg(T_odom_base);

	// the convention is that the 
	odom_msg.pose.pose.position.x = odom_from_base_tf.translation.x;
	odom_msg.pose.pose.position.y = odom_from_base_tf.translation.y;
	odom_msg.pose.pose.position.z = odom_from_base_tf.translation.z;
	odom_msg.pose.pose.orientation = odom_from_base_tf.rotation;
	odom_msg.header.frame_id = tf_odom_frame;  // e.g. jackal1_tf/odom
	odom_msg.child_frame_id  = tf_child_frame; // e.g. jackal1_tf/base_link
	
	odom_msg.header.stamp    = ros::Time::now();

	// --- Broadcast TF: odom → base_link ---
	static tf2_ros::TransformBroadcaster br2;
	geometry_msgs::TransformStamped odom_from_base_tf_msg;
	odom_from_base_tf_msg.header.stamp 		= odom_msg.header.stamp;
	odom_from_base_tf_msg.header.frame_id 	= tf_odom_frame; // e.g. jackal1_tf/odom
	odom_from_base_tf_msg.child_frame_id  	= tf_base_frame;  // e.g. jackal1_tf/base_link
	odom_from_base_tf_msg.transform      	= odom_from_base_tf;

	br2.sendTransform(odom_from_base_tf_msg);

	
	
	 // Build the lightweight “state” pose_msg (roll/pitch/yaw + planar speed) – legacy encoding
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";  // if you really want map here
	pose_msg.header.stamp = ros::Time::now();

	tf2::Quaternion q2;
	q2.setX(pos.orientation.x);
	q2.setY(pos.orientation.y);
	q2.setZ(pos.orientation.z);
	q2.setW(pos.orientation.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q2).getRPY(roll, pitch, yaw);
	if (std::isnan(yaw)) { roll = pitch = yaw = 0.0; }

	pose_msg.pose.orientation.x = roll;
	pose_msg.pose.orientation.y = pitch;
	pose_msg.pose.orientation.z = yaw;
	pose_msg.pose.orientation.w = 0.0;

	pose_msg.pose.position.x = pos.position.x;
	pose_msg.pose.position.y = pos.position.y;
	double planar_speed = std::hypot(msg.twist[index].linear.x, msg.twist[index].linear.y);
	pose_msg.pose.position.z = planar_speed;

	state_pub_.publish(pose_msg);
    odom_pub_.publish(odom_msg);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_robot_state_publisher_node");
	ros::NodeHandle n;
	ros::Subscriber robot_state_sub_;

	if (!n.getParam(ros::this_node::getName() + "/rate", rate_))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/rate not set");
		return 0;
	}

	string root_frame;
	if (!n.getParam(ros::this_node::getName() + "/root_frame", root_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/root_frame not set");
		return 0;
	}

	string base_frame;
	if (!n.getParam(ros::this_node::getName() + "/base_frame", base_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/base_frame not set");
		return 0;
	}

	string robot_state_topic;
	if (!n.getParam(ros::this_node::getName() + "/robot_state_topic", robot_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/robot_state_topic not set");
		return 0;
	}
	

	string vel_state_topic;
	if (!n.getParam(ros::this_node::getName() + "/vel_state_topic", vel_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/vel_state_topic not set");
		return 0;
	}
	

	// robot_namespace is set here as a global prameter 
	if (!n.getParam(ros::this_node::getName() + "/robot_namespace", robot_namespace)) {
    ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter "
                     << ros::this_node::getName() << "/robot_namespace not set");
    return 0;
	}

	if (!n.getParam(ros::this_node::getName() + "/tf_prefix", tf_prefix)) {
    ROS_ERROR("tf_prefix not set");
    return 0;
	}
	// Set the linkes and frames
	target_link = robot_namespace + "::base_link";          // e.g., "jackal1::base_link"
	tf_base_frame 	= tf_prefix + "/base_link";           // TF frame, e.g., "jackal1/base_link"
	tf_odom_frame 	= tf_prefix + "/odom";                // e.g., "jackal1/odom"
	tf_child_frame =  tf_base_frame; 								// for odometry.child_frame_id

	robot_state_sub_ = n.subscribe(vel_state_topic, 1, VelocityCallBack);

	state_pub_ = n.advertise<geometry_msgs::PoseStamped>(robot_state_topic, 1);
	odom_pub_ = n.advertise<nav_msgs::Odometry>("odometry/filtered", 1);

	ros::ServiceClient link_state_client_ = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

	gazebo_msgs::SetLinkState link;
	link.request.link_state.link_name = "base_link";

	tf2_ros::TransformListener tfListener(tfBuffer);
	last_velocity_callback_ = ros::Time::now();
	ros::spin();

	
}
