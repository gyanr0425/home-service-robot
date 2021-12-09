#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


  while (ros::ok()) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
  
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;
  
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -0.5;
    marker.pose.position.y = 1.5;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
  
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
  
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Publish a marker with initial setup
    marker_pub.publish(marker);
    ROS_INFO("[add_markers] Let's pick up");
    sleep(5);

    // Change action to delete the marker and publish
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("[add_markers] object was picked up");
    sleep(5);

    // Change action to add a new marker and publish
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3.5;
    marker.pose.position.y = -0.5;
    marker_pub.publish(marker);
    ROS_INFO("[add_markers] object was dropped");
    sleep(5);

    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker); 
    ROS_INFO("[add_markers] session end");

    loop_rate.sleep();

    ros::spinOnce();
  }
}

