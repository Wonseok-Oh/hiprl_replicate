/*
 * knolwedge_updater.h
 *
 *     Created on: May 17, 2021
 *         Author: morin
 */
#ifndef HIPRL_REPLICATE_INCLUDE_HIPRL_REPLICATE_KNOWLEDGE_UPDATER_H_
#define HIPRL_REPLICATE_INCLUDE_HIPRL_REPLICATE_KNOWLEDGE_UPDATER_H_
#include "ros/ros.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/processReset.h"

#include "hiprl_replicate/ActionExecution.h"
#include "hiprl_replicate/Obs.h"
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <string>
namespace knowledge_updater {

enum class Knowledge_type {
	instance,
	fact,
	function,
	expression,
	inequality
};

enum class Update_type {
	add_knowledge,
	add_goal,
	remove_knowledge,
	remove_goal,
	add_metric,
	remove_metric
};

class KnowledgeUpdater {
public:
	KnowledgeUpdater(std::string process_num);
	~KnowledgeUpdater();
	void observationCallback(const hiprl_replicate::Obs::ConstPtr& msg);
	void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
	void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
	void resetCallback(const rosplan_knowledge_msgs::processReset::ConstPtr &msg);
	bool check_precondition(hiprl_replicate::ActionExecution::Request &req,
			hiprl_replicate::ActionExecution::Response &res);
	bool process_action_effect(hiprl_replicate::ActionExecution::Request &req,
			hiprl_replicate::ActionExecution::Response &res);
private:
	ros::NodeHandle m_nh, m_private_nh;
	ros::Subscriber m_obs_sub, m_pose_sub, m_goal_sub, m_reset_sub;
	ros::ServiceClient m_client, m_query_domain_client, m_query_knowledge_client, m_instance_client;
	ros::ServiceServer m_precondition_checker, m_action_effect_processor;
	std::vector<int> m_seen_box_id_list;
	std::vector<int> m_checked_box_id_list;
	std::vector<int> m_seen_obj_id_list;
	std::string m_kb, m_goal_pose;
	int m_resolution, m_grid_size, m_prev_pos_x, m_prev_pos_y;
	bool m_search_mode, m_keep_mode, m_bring_mode;

};

}

#endif /* HIPRL_REPLICATE_INCLUDE_HIPRL_REPLICATE_KNOWLEDGE_UPDATER_H_ */
