#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <point_cloud_selector/SelectCylinder.h>
#include <point_cloud_selector/SetCylinderProperties.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <string>

bool g_button_pressed = false;
bool g_last_button_state = false;

visualization_msgs::Marker g_cylinder_marker;

void spacenavJoyCallback(const sensor_msgs::JoyConstPtr joy_msg)
{
	if(g_last_button_state && !joy_msg->buttons[0] && !joy_msg->buttons[1])
	{
		g_button_pressed = true;
	}

	g_last_button_state = joy_msg->buttons[0] || joy_msg->buttons[1];
}

bool setCylinderPropertiesCallback(point_cloud_selector::SetCylinderProperties::Request &req,
								   point_cloud_selector::SetCylinderProperties::Response &res)
{
	g_cylinder_marker.scale.x = req.radius;
	g_cylinder_marker.scale.y = req.radius;
	g_cylinder_marker.scale.z = req.height;

	g_cylinder_marker.color = req.color;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cylinder_selector");

	std::string node_name = ros::this_node::getName();
	if(!node_name.empty())
		node_name = "cylinder_selector";
		
	ros::NodeHandle nh;
	ros::NodeHandle pcs_nh("point_cloud_selector");
	ros::NodeHandle this_nh(node_name);
	
	ros::Publisher g_cylinder_marker_pub = pcs_nh.advertise<visualization_msgs::Marker>( node_name, 0 );

	ros::ServiceServer set_cylinder_props_srv = this_nh.advertiseService("cylinder_properties", setCylinderPropertiesCallback);

	ros::ServiceClient select_cylinder_client = nh.serviceClient<point_cloud_selector::SelectCylinder>("cylinder_properties_server");
	ROS_INFO("waiting for the existence of the service");
	select_cylinder_client.waitForExistence();
	ROS_INFO("service found");
	ros::Subscriber joy_sub = nh.subscribe("spacenav/joy", 1, spacenavJoyCallback);

	tf::TransformListener tf_listener;

	g_cylinder_marker.header.frame_id = "pelvis";
	g_cylinder_marker.ns = "point_cloud_selector";
	g_cylinder_marker.id = 0;
	g_cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
	g_cylinder_marker.action = visualization_msgs::Marker::ADD;

	g_cylinder_marker.pose.orientation.w = 1.0;

	g_cylinder_marker.scale.x = 1.0;
	g_cylinder_marker.scale.y = 1.0;
	g_cylinder_marker.scale.z = 1.0;

	g_cylinder_marker.color.a = .5;
	g_cylinder_marker.color.g = 1.0;

	point_cloud_selector::SelectCylinder select_cylinder_msg;

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		tf::StampedTransform transform;

		try {
			tf_listener.lookupTransform("pelvis", "cylinder_cursor", ros::Time(0), transform);
			geometry_msgs::Vector3 position;
			tf::vector3TFToMsg(transform.getOrigin(), position);
			g_cylinder_marker.pose.position.x = position.x;
			g_cylinder_marker.pose.position.y = position.y;
			g_cylinder_marker.pose.position.z = position.z+(g_cylinder_marker.scale.z/2.0);
			tf::quaternionTFToMsg(transform.getRotation(), g_cylinder_marker.pose.orientation);

		} catch( const tf::TransformException& except) {
			ROS_ERROR("%s", except.what());
		}

		g_cylinder_marker.header.stamp = ros::Time();
		g_cylinder_marker_pub.publish(g_cylinder_marker);

		if(g_button_pressed)
		{
			select_cylinder_msg.request.pose = g_cylinder_marker.pose;
			select_cylinder_msg.request.radius = g_cylinder_marker.scale.x;
			select_cylinder_msg.request.height = g_cylinder_marker.scale.z;
			select_cylinder_client.call(select_cylinder_msg);
			g_button_pressed = false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
