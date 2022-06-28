//
// Created by handleandwheel on 2021/10/6.
//

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "math.h"

using namespace std;

geometry_msgs::Point increase;

ros::Publisher quat_pub;

geometry_msgs::Quaternion exp_q(const geometry_msgs::Point &r_v)
{
	geometry_msgs::Quaternion output;
	double norm2 = sqrt(r_v.x*r_v.x+r_v.y*r_v.y+r_v.z*r_v.z);
	double param = sin(norm2) / norm2;
	output.w = cos(norm2);
	output.x = r_v.x * param;
	output.y = r_v.y * param;
	output.z = r_v.z * param;
	return output;
}

geometry_msgs::Quaternion odot(const geometry_msgs::Quaternion &left, const geometry_msgs::Quaternion &right)
{
	geometry_msgs::Quaternion output;
	output.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.y;
	output.x = left.w * right.x + right.w * left.x + left.y * right.z - left.z * right.y;
	output.y = left.w * right.y + right.w * left.y + left.z * right.x - left.x * right.z;
	output.z = left.w * right.z + right.w * left.z + left.x * right.y - left.y * right.x;
	return output;
}

void quat_callback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
	geometry_msgs::Quaternion quaternion;
	
	quaternion.w = msg->w;
	quaternion.x = msg->x;
	quaternion.y = msg->y;
	quaternion.z = msg->z;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(quaternion, quat);
	
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	ROS_INFO("quat1:%f, quat2:%f, quat3:%f, quat4:%f", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
	ROS_INFO("increase:%f, roll:%f, pitch:%f, yaw:%f", increase.x, roll, pitch, yaw);
	
	// add increasement
	geometry_msgs::Quaternion exp_eta = exp_q(increase);
	quaternion = odot(exp_eta, quaternion);
	
	quat_pub.publish(quaternion);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quaternion_test_node");
	ros::NodeHandle nh;
	
	increase.x = 0.1;
	increase.y = 0.0;
	increase.z = 0.0;
	
	ros::Subscriber quat_sub = nh.subscribe<geometry_msgs::Quaternion>("/quat", 1, quat_callback);
	
	quat_pub = nh.advertise<geometry_msgs::Quaternion>("/quat", 1);
	
	geometry_msgs::Quaternion quaternion;
	
	quaternion.w = 1;
	quaternion.x = 0;
	quaternion.y = 0;
	quaternion.z = 0;
	
	quat_pub.publish(quaternion);
	
	ros::spin();
	
	ros::waitForShutdown();
	
	return 0;
}