#include "hiprl_replicate/hiprl_action_interface.h"

/* The implementation of hiprl_action_interface.h */
namespace KCL_rosplan {
	HIPRLActionInterface::HIPRLActionInterface(ros::NodeHandle &nh){}
	bool HIPRLActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
		// complete the action
		ROS_INFO("KCL: (%s) Action completing.", msg->name.c_str());
		return true;
	}

}
