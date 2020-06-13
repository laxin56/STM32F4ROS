#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <math.h>
tf::Matrix3x3 obs_mat;
tf::Quaternion q_tf;
visualization_msgs::Marker marker;
ros::Publisher marker_pub;
float x = 0, y = 0, ang = 0, rad = 0;

void Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  
  x += (msg->linear_acceleration.x)/1000000000000;
  y += (msg->linear_acceleration.y)/1000000000000;
  ang = (msg->linear_acceleration.z)/10000;
  rad = (3.14/180)*ang;
  
  // ROS_INFO("I heard %f", ang);
    ROS_INFO("I heard %f", rad);
    marker.ns = "basic_shapes";
    marker.id = 0;
    
    if (ang < 100 && ang > 100){
      obs_mat.setEulerYPR(rad,0,0);
      obs_mat.getRotation(q_tf);
      marker.pose.orientation.z = q_tf.getZ();
      marker.pose.orientation.x = q_tf.getX();
      marker.pose.orientation.y = q_tf.getY();
      marker.pose.orientation.w = q_tf.getW();
      ROS_INFO("I heard %f", marker.pose.orientation.z);
    }
   
    marker.pose.position.x = 40+x;
    marker.pose.position.y = 28+y;
    marker_pub.publish(marker);
 
}

int main( int argc, char** argv )
{
  
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
   marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // while (ros::ok())
  //  {
      
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/my_frame";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
 
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0.26;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
  
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
  
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
 
      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);

      ros::Subscriber sub = n.subscribe("imu", 100, Callback);
      ros::spin();
   
 
      //  r.sleep();
      // }
      return 0;
}
