#include "ros/ros.h"
#include "../include/nautonomous_navigation_pathfinder/FindPathAmsterdamCanals.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigate_amsterdam_canals");
	
	float current[] = {52.36727, 4.93093};
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<nautonomous_navigation_pathfinder::FindPathAmsterdamCanals>("find_path_amsterdam_canals");
	nautonomous_navigation_pathfinder::FindPathAmsterdamCanals srv;
	srv.request.goalLatitude = 52.36905;
	srv.request.goalLatitude = 4.89248;
	while (ros::ok())
	{
		if (client.call(srv))
		{
			int x = srv.response.x;
			int y = srv.response.y;
			float lat = srv.response.lat;
			float lon = srv.response.lon;
			
			ROS_INFO("x: %d, y: %d / lat: %f, lon: %f", x, y, lat, lon);
		}
		else
		{
			ROS_ERROR("Failed to call service add_two_ints");
			return 1;
		}
	}

	return 0;
}
