#include "ros/ros.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "hiprl_replicate/knowledge_updater.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "update_knowledge_client");
	knowledge_updater::KnowledgeUpdater knowledge_updater;
	ros::spin();
	return 0;
}
