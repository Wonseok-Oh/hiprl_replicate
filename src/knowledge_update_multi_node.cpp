#include "ros/ros.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "hiprl_replicate/knowledge_updater_multi.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "update_knowledge_client");
	std::string process_num = std::string(argv[1]);
	knowledge_updater_multi::KnowledgeUpdaterMulti knowledge_updater_multi(process_num);
	ros::spin();
	return 0;
}
