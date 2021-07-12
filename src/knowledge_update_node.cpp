#include "ros/ros.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "hiprl_replicate/knowledge_updater.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "update_knowledge_client");
	std::string process_num = std::string(argv[1]);
	knowledge_updater::KnowledgeUpdater knowledge_updater(process_num);
	ros::spin();
	return 0;
}
