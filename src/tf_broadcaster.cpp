#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  
  /****************** Access to physical parameters of eod ********************/ 
  double bk_x, bk_y, bk_z; // Base-Kinect x,y,z

  bool loaded = 1;
  
  loaded = ros::param::get("/eod/tf/base_kinect/dx", bk_x);
  loaded = loaded & ros::param::get("/eod/tf/base_kinect/dy", bk_y);
  loaded = loaded & ros::param::get("/eod/tf/base_kinect/dz", bk_z);
  
  // ROS_DEBUG("eod/tf/base_kinect/dx: %1.2f", bk_x);
  // ROS_DEBUG("eod/tf/base_kinect/dy: %1.2f", bk_x);
  // ROS_DEBUG("eod/tf/base_kinect/dz: %1.2f", bk_x);
  
  if(!loaded) ROS_ERROR("Failure in loading tf parameters");
  
  
  
  /************************* Sending transformations ************************/
	while(n.ok()){  
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(bk_x, bk_y, bk_z)),
        ros::Time::now(), "base_link", "base_kinect"));
    r.sleep();
  }

}