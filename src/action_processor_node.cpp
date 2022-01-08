#include "ros/ros.h"
#include "std_msgs/String.h"

#include "action_processor_node/Action.h"
#include "action_processor_node/ActiveActions.h"
#include "action_processor_node/RequestActionArray.h"
#include "action_processor_node/ActionResponse.h"
#include "action_processor_node/ParallelActionArray.h"
#include "action_processor_node/SeriesActionArray.h"

#include <thread>
#include <string>
#include <mutex>

ros::NodeHandle* node;

void requestActionCallback(const action_processor_node::RequestActionArray &msg)
{
	(void)msg;
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "action_processor_node");

	ros::NodeHandle n;

	node = &n;

	ros::Subscriber motorControl = node->subscribe("RequestAction", 100, requestActionCallback);




	ros::spin();
	return 0;
}