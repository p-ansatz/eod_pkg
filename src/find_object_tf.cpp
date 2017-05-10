#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include <geometry_msgs/PoseStamped.h>

class TfExample
{
public:
	TfExample() :
		mapFrameId_("map"),
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

		ros::NodeHandle nh;
		subs_ = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
		pub = nh.advertise <geometry_msgs::PoseStamped>("objects_stamped_poses", 1000);
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		if(msg->objects.data.size())
		{
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];
				std::string objectFrameId = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString(); // "object_1", "object_2"

				tf::StampedTransform pose;
				tf::StampedTransform poseCam;
				try
				{
					// Get transformation from "object_#" frame to target frame "map"
					// The timestamp matches the one sent over TF
					tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, pose);
					tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
				}
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}

				// Here "pose" is the position of the object "id" in "/map" frame.
				ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, mapFrameId_.c_str(),
						pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
				ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, msg->header.frame_id.c_str(),
						poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
						poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());
			
				// pubblicazione della posa sul topic objects_stamped_poses
				geometry_msgs::PoseStamped object_pose;
			    object_pose.header.stamp = ros::Time::now();
			    object_pose.header.frame_id = mapFrameId_.c_str();
			    object_pose.pose.position.x = pose.getOrigin().x();
			    object_pose.pose.position.y = pose.getOrigin().y();
			    object_pose.pose.position.z = pose.getOrigin().z();  
			    object_pose.pose.orientation.x = pose.getRotation().x();
			    object_pose.pose.orientation.y = pose.getRotation().y();
			    object_pose.pose.orientation.z = pose.getRotation().z();
			    object_pose.pose.orientation.w = pose.getRotation().w();

				pub.publish(object_pose);
			}
		}
	}

private:
	std::string mapFrameId_;
	std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
    ros::Publisher pub;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "find_object_tf");

    TfExample sync;
    ros::spin();
}
