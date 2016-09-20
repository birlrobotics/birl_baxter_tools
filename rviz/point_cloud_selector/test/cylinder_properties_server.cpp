#include <ros/ros.h>
#include <point_cloud_selector/SelectCylinder.h>

bool setCylinderPropertiesCallback(point_cloud_selector::SelectCylinder::Request &req,
								   point_cloud_selector::SelectCylinder::Response &res)
{
	ROS_INFO_STREAM("Received the property settings:\n" << req.pose << "\nradius: " << req.radius \
					<< "\nheight: " << req.height);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cylinder_properties_server");
	ros::NodeHandle nh;

	ros::ServiceServer set_cylinder_props_srv = nh.advertiseService("cylinder_properties_server", setCylinderPropertiesCallback);

	ros::spin();
}
