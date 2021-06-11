#include "hiprl_replicate/hiprl_action_interface.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "rosplan_action");
	ros::NodeHandle nh("~");

	// create PDDL action subscriber
	KCL_rosplan::HIPRLActionInterface hai(nh);
	hai.runActionInterface();
	return 0;
}
