#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class PublishMarker
{

private:

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odometry_sub;
  visualization_msgs::Marker marker;
  
public:

  PublishMarker() {
    
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    odometry_sub = n.subscribe("/odom", 1, &PublishMarker::funcCallback, this);
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. 
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type as CUBE
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -3.5;
    marker.pose.position.y = 7.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    
  }
  void funcCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    float x_pos             = msg->pose.pose.position.x;
    float y_pos             = msg->pose.pose.position.y;

    if ((x_pos < -3.0) && (x_pos > -4.0) && (y_pos < 7.5) && (y_pos > 6.5)) {
     
      ROS_INFO("Reached Pickup Goal");
      ros::Duration(5).sleep();

      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Removing marker");
      marker_pub.publish(marker);  
  
    }
    else if ((x_pos < -6.5) && (x_pos > -7.5) && (y_pos < 0.5) && (y_pos > -0.5)) {
        
        ROS_INFO("Reached Dropoff Goal");
        
        marker.pose.position.x = -7.0;
        marker.pose.position.y = 0.0;
        marker.action = visualization_msgs::Marker::ADD;

        marker_pub.publish(marker);
        ros::Duration(5).sleep();
        
      }
    }
};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  
  PublishMarker publishing_marker;
  ros::spin();

  return 0; 
}
