#include <ros/ros.h>
#include <vector>

#include <rosplan_action_interface/RPActionInterface.h>
#include "hiprl_replicate/ActionExecution.h"

#ifndef HIPRL_ACTION_INTERFACE_H_
#define HIPRL_ACTION_INTERFACE_H_

  /*
   * This file defines an action interface for HiP-RL in 2D gym Grid Environment
   */
namespace KCL_rosplan {
	class HIPRLActionInterface: public RPActionInterface {
	public:
		/* constructor */
		HIPRLActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

	};
}


#endif
