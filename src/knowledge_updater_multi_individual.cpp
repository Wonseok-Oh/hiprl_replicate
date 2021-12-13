#include "hiprl_replicate/knowledge_updater_multi_individual.h"

using namespace knowledge_updater_multi_individual;
using namespace std;

KnowledgeUpdaterMultiIndividual::KnowledgeUpdaterMultiIndividual(std::string process_num_agent_num):
		m_resolution(3), m_grid_size(20), m_search_mode(true), m_keep_mode(false), m_bring_mode(false),
		m_init(false), m_agent_pos_x(-1), m_agent_pos_y(-1), m_prev_pos_x(-1), m_prev_pos_y(-1),
		m_total_agent_num(0){
	m_private_nh = ros::NodeHandle("~");
	m_nh = ros::NodeHandle();
	m_kb = string("knowledge_base");
	m_private_nh.getParam("knowledge_base", m_kb);
	m_private_nh.getParam("total_agent_num", m_total_agent_num);
	stringstream ss;
	ss << "/" << m_kb << "/query_state";
	m_query_knowledge_client = m_nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(ss.str());
	ss.str("");
	ss << "/" << m_kb << "/domain/operator_details";
	m_query_domain_client = m_nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
	ss.str("");
	ss << "/" << m_kb << "/state/instances";
	istringstream process_num_stream(process_num_agent_num);
	string stringBuffer;
	vector<string> num_vector;

	while(getline(process_num_stream, stringBuffer, '_')){
		num_vector.push_back(stringBuffer);
		cout << stringBuffer << " ";
	}
	m_process_num = stringBuffer[0];
	m_agent_num = stringBuffer[1];

	m_instance_client = m_nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(ss.str());
	m_obs_sub = m_nh.subscribe<hiprl_replicate::Obs_multi>("observation" + m_process_num, 1000, &KnowledgeUpdaterMultiIndividual::observationCallback, this);
	m_goal_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("goal_pose" + m_process_num, 1, &KnowledgeUpdaterMultiIndividual::goalCallback, this);
	m_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("pose" + process_num_agent_num, 1, &KnowledgeUpdaterMultiIndividual::poseCallback, this);
	ss.str("");
	ss << "/" << m_kb << "/reset";
	m_reset_sub = m_nh.subscribe<rosplan_knowledge_msgs::processReset>(ss.str(), 1, &KnowledgeUpdaterMultiIndividual::resetCallback, this);
	ss.str("");
	ss << "/" << m_kb << "/update_array";
	m_client = m_nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());
	m_seen_box_id_list = std::vector<int>();
	m_checked_box_id_list = std::vector<int>();
	m_seen_obj_id_list = std::vector<int>();
	m_precondition_checker = m_nh.advertiseService("check_precondition" + process_num_agent_num, &KnowledgeUpdaterMultiIndividual::check_precondition, this);
	m_action_effect_processor = m_nh.advertiseService("process_action_effect" + process_num_agent_num, &KnowledgeUpdaterMultiIndividual::process_action_effect, this);
}

KnowledgeUpdaterMultiIndividual::~KnowledgeUpdaterMultiIndividual(){
}

void KnowledgeUpdaterMultiIndividual::resetCallback(const rosplan_knowledge_msgs::processReset::ConstPtr &msg){
	m_search_mode = true;
	m_keep_mode = false;
	m_bring_mode = false;
	m_prev_pos_x = -1;
	m_prev_pos_y = -1;
	m_seen_box_id_list.clear();
	m_checked_box_id_list.clear();
	m_seen_obj_id_list.clear();
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
	m_init = false;

	srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
	diagnostic_msgs::KeyValue value;
	rosplan_knowledge_msgs::KnowledgeItem item;
	item.knowledge_type = static_cast<int>(Knowledge_type::fact);
	item.attribute_name.assign(string("handsfree"));
	value.key.assign(string("a"));
	value.value.assign(string("robot") + m_agent_num);
	item.values.push_back(value);
	srv.request.knowledge.push_back(item);

	srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
	item = rosplan_knowledge_msgs::KnowledgeItem();
	item.knowledge_type = static_cast<int>(Knowledge_type::instance);
	item.instance_type.assign(string("agent"));
	item.instance_name.assign(string("robot") + m_agent_num);
	srv.request.knowledge.push_back(item);

	while (m_client.call(srv) != true){
		ROS_ERROR("KnowledgeUpdaterMultiIndividual::poseCallback: Failed to call service knowledge_update");
		sleep(1);
	}


}

bool KnowledgeUpdaterMultiIndividual::process_action_effect(hiprl_replicate::ActionExecution::Request &req,
										hiprl_replicate::ActionExecution::Response &res){
	ROS_INFO("Process_action_effect start");
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> sensed_predicates;


	rosplan_knowledge_msgs::DomainFormula params;
	rosplan_knowledge_msgs::DomainOperator op;


	// fetch action params
	std::stringstream ss;
	ss << "/" << m_kb << "/domain/operator_details";
	ros::service::waitForService(ss.str(),ros::Duration(20));
	rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;

	params.name = req.name;
	srv.request.name = params.name;
	ROS_INFO_STREAM("parameter name: " << params.name);
	if(m_query_domain_client.call(srv)) {
		params = srv.response.op.formula;
		op = srv.response.op;
	} else {
		ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
		res.success = false;
		return false;
	}

	// check PDDL parameters
	std::vector<bool> found(params.typed_parameters.size(), false);
	std::map<std::string, std::string> boundParameters;
	for(size_t j=0; j<params.typed_parameters.size(); j++) {
		for(size_t i=0; i<req.parameters.size(); i++) {
			if(params.typed_parameters[j].key == req.parameters[i].key) {
				boundParameters[req.parameters[i].key] = req.parameters[i].value;
				found[j] = true;
				break;
			}
		}
		if(!found[j]) {
			ROS_INFO("KCL: (%s) aborting action dispatch; malformed parameters, missing %s", params.name.c_str(), params.typed_parameters[j].key.c_str());
			res.success = false;
			return false;
		}
	}


	// collect predicates from operator description
	std::vector<std::string> predicateNames;

	// effects
	std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_start_add_effects.begin();
	for(; pit!=op.at_start_add_effects.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.at_start_del_effects.begin();
	for(; pit!=op.at_start_del_effects.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.at_end_add_effects.begin();
	for(; pit!=op.at_end_add_effects.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.at_end_del_effects.begin();
	for(; pit!=op.at_end_del_effects.end(); pit++)
		predicateNames.push_back(pit->name);

	// simple conditions
	pit = op.at_start_simple_condition.begin();
	for(; pit!=op.at_start_simple_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.over_all_simple_condition.begin();
	for(; pit!=op.over_all_simple_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.at_end_simple_condition.begin();
	for(; pit!=op.at_end_simple_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	// negative conditions
	pit = op.at_start_neg_condition.begin();
	for(; pit!=op.at_start_neg_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.over_all_neg_condition.begin();
	for(; pit!=op.over_all_neg_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	pit = op.at_end_neg_condition.begin();
	for(; pit!=op.at_end_neg_condition.end(); pit++)
		predicateNames.push_back(pit->name);

	// fetch and store predicate details
	ss.str("");
	ss << "/" << m_kb << "/domain/predicate_details";
	ros::service::waitForService(ss.str(),ros::Duration(20));
	ros::ServiceClient predClient = m_nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(ss.str());
	std::vector<std::string>::iterator nit = predicateNames.begin();
	for(; nit!=predicateNames.end(); nit++) {
		if (predicates.find(*nit) != predicates.end()) continue;
		if (*nit == "=" || *nit == ">" || *nit == "<" || *nit == ">=" || *nit == "<=") continue;
		rosplan_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
		predSrv.request.name = *nit;
		if(predClient.call(predSrv)) {
			if (predSrv.response.is_sensed){
				sensed_predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));
			} else {
				predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));
			}
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for predicate details, %s", params.name.c_str());
			return false;
		}
	}




	// update knowledge base
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

	// simple START del effects
	for(int i=0; i<op.at_start_del_effects.size(); i++) {

		std::map<std::string, rosplan_knowledge_msgs::DomainFormula>::iterator it = sensed_predicates.find(op.at_start_del_effects[i].name);
		if(it != sensed_predicates.end()) continue; // sensed predicate

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		item.attribute_name = op.at_start_del_effects[i].name;
		item.values.clear();
		diagnostic_msgs::KeyValue pair;
		for(size_t j=0; j<op.at_start_del_effects[i].typed_parameters.size(); j++) {
			pair.key = predicates[op.at_start_del_effects[i].name].typed_parameters[j].key;
			pair.value = boundParameters[op.at_start_del_effects[i].typed_parameters[j].key];
			item.values.push_back(pair);
		}
		updatePredSrv.request.knowledge.push_back(item);
		updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
	}

	// simple START add effects
	for(int i=0; i<op.at_start_add_effects.size(); i++) {

		std::map<std::string, rosplan_knowledge_msgs::DomainFormula>::iterator it = sensed_predicates.find(op.at_start_add_effects[i].name);
		if(it != sensed_predicates.end()) continue; // sensed predicate

		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		item.attribute_name = op.at_start_add_effects[i].name;
		item.values.clear();
		diagnostic_msgs::KeyValue pair;
		for(size_t j=0; j<op.at_start_add_effects[i].typed_parameters.size(); j++) {
			pair.key = predicates[op.at_start_add_effects[i].name].typed_parameters[j].key;
			pair.value = boundParameters[op.at_start_add_effects[i].typed_parameters[j].key];
			item.values.push_back(pair);
		}
		updatePredSrv.request.knowledge.push_back(item);
		updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
	}

	if(updatePredSrv.request.knowledge.size()>0 && !m_client.call(updatePredSrv))
		ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());



	// simple END del effects
	for(int i=0; i<op.at_end_del_effects.size(); i++) {
		std::map<std::string, rosplan_knowledge_msgs::DomainFormula>::iterator it = sensed_predicates.find(op.at_end_del_effects[i].name);
		if(it != sensed_predicates.end()) continue; // sensed predicate
		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		item.attribute_name = op.at_end_del_effects[i].name;
		item.values.clear();
		diagnostic_msgs::KeyValue pair;
		for(size_t j=0; j<op.at_end_del_effects[i].typed_parameters.size(); j++) {
			pair.key = predicates[op.at_end_del_effects[i].name].typed_parameters[j].key;
			pair.value = boundParameters[op.at_end_del_effects[i].typed_parameters[j].key];
			item.values.push_back(pair);
		}
		updatePredSrv.request.knowledge.push_back(item);
		updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
	}

	// simple END add effects
	for(int i=0; i<op.at_end_add_effects.size(); i++) {
		std::map<std::string, rosplan_knowledge_msgs::DomainFormula>::iterator it = sensed_predicates.find(op.at_end_add_effects[i].name);
		if(it != sensed_predicates.end()) continue; // sensed predicate
		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		item.attribute_name = op.at_end_add_effects[i].name;
		item.values.clear();
		diagnostic_msgs::KeyValue pair;
		for(size_t j=0; j<op.at_end_add_effects[i].typed_parameters.size(); j++) {
			pair.key = predicates[op.at_end_add_effects[i].name].typed_parameters[j].key;
			pair.value = boundParameters[op.at_end_add_effects[i].typed_parameters[j].key];
			item.values.push_back(pair);
		}
		updatePredSrv.request.knowledge.push_back(item);
		updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
	}

	ROS_INFO("updatePredSrv request knowledge size: %d", static_cast<int>(updatePredSrv.request.knowledge.size()));
	if(updatePredSrv.request.knowledge.size()>0 && !m_client.call(updatePredSrv))
		ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());

	ROS_INFO("Process_action_effect end");
	return true;
}

bool KnowledgeUpdaterMultiIndividual::check_precondition(hiprl_replicate::ActionExecution::Request &req,
										hiprl_replicate::ActionExecution::Response &res){
    // get domain operator details
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
    srv.request.name = req.name;
    if(!m_query_domain_client.call(srv)) {
        ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), req.name.c_str());
        return false;
    }

    // setup service call
    rosplan_knowledge_msgs::DomainOperator op = srv.response.op;
    rosplan_knowledge_msgs::KnowledgeQueryService positiveQuerySrv;

    // iterate through conditions
    std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit = op.at_start_simple_condition.begin();
    for(; cit!=op.at_start_simple_condition.end(); cit++) {

        // create condition
        rosplan_knowledge_msgs::KnowledgeItem condition;
        condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        condition.attribute_name = cit->name;

        // populate parameters
        for(int i=0; i<cit->typed_parameters.size(); i++) {

            // set parameter label to predicate label
            diagnostic_msgs::KeyValue param;
            param.key = cit->typed_parameters[i].key;

            // search for correct operator parameter value
            for(int j=0; j<req.parameters.size() && j<op.formula.typed_parameters.size(); j++) {
                if(op.formula.typed_parameters[j].key == cit->typed_parameters[i].key) {
                    param.value = req.parameters[j].value;
                }
            }
            condition.values.push_back(param);
        }
        positiveQuerySrv.request.knowledge.push_back(condition);
    }

    // checking negative preconditions
    cit = op.at_start_neg_condition.begin();
    // flag to indicate that at least one of the negative conditions was found in KB
    bool neg_preconditions = false;
    rosplan_knowledge_msgs::KnowledgeQueryService negativeQuerySrv;
    for(; cit!=op.at_start_neg_condition.end(); cit++) {
        // create condition
        rosplan_knowledge_msgs::KnowledgeItem condition;
        condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        condition.attribute_name = cit->name;

        // populate parameters
        for(int i=0; i<cit->typed_parameters.size(); i++) {

            // set parameter label to predicate label
            diagnostic_msgs::KeyValue param;
            param.key = cit->typed_parameters[i].key;

            // search for correct operator parameter value
            for(int j=0; j<req.parameters.size() && j<op.formula.typed_parameters.size(); j++) {
                if(op.formula.typed_parameters[j].key == cit->typed_parameters[i].key) {
                    param.value = req.parameters[j].value;
                }
            }
            condition.values.push_back(param);
        }
        negativeQuerySrv.request.knowledge.push_back(condition);
    }

    // check negative conditions in knowledge base (conditions that should not be present in KB)
    if (m_query_knowledge_client.call(negativeQuerySrv)) {
        // iterate over results to check if at least one fact was found in KB
        for(auto it = negativeQuerySrv.response.results.begin(); it != negativeQuerySrv.response.results.end(); it++)
            if (*it) {
                // rise flag to indicate that at least one neg condition was not achieved
                neg_preconditions = true;

                // print which negative condition was found in KB
                std::stringstream ss;
                // get index of failed neg precondition
                int index = std::distance(it, negativeQuerySrv.response.results.begin());
                ss << negativeQuerySrv.request.knowledge[index].attribute_name;
                auto pit = negativeQuerySrv.request.knowledge[index].values.begin();
                for(; pit != negativeQuerySrv.request.knowledge[index].values.end(); pit++) {
                    ss << " ";
                    ss << pit->value.c_str();
                }

                ROS_INFO("Negative precondition not achieved : not (%s)", ss.str().c_str());
                res.success = false;
            }
    } else {
        ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
        return false;
    }

    // check positive conditions in knowledge base
    if (m_query_knowledge_client.call(positiveQuerySrv)) {

        if(!positiveQuerySrv.response.all_true) {
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit;
            for(kit=positiveQuerySrv.response.false_knowledge.begin(); kit != positiveQuerySrv.response.false_knowledge.end(); kit++)
                ROS_INFO("KCL: (%s) Precondition not achieved: %s", ros::this_node::getName().c_str(), kit->attribute_name.c_str());
            	res.success = false;
        }
        res.success = positiveQuerySrv.response.all_true && !neg_preconditions;
        return true;

    } else {
        ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
        return false;
    }
}

void KnowledgeUpdaterMultiIndividual::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg){
	m_goal_pose = string("f") + to_string(static_cast<int>(msg->pose.position.y)) + string("-") + to_string(static_cast<int>(msg->pose.position.x)) + string("f");
}

void KnowledgeUpdaterMultiIndividual::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
	rosplan_knowledge_msgs::KnowledgeItem item;

	if (!m_init){
		m_init = true;

		cout << "init knowledge update in poseCallback" << endl;
		// add handsfree knowledge
		srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
		diagnostic_msgs::KeyValue value;
		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = static_cast<int>(Knowledge_type::fact);
		item.attribute_name.assign(string("handsfree"));
		value.key.assign(string("a"));
		value.value.assign(string("robot") + m_agent_num);
		item.values.push_back(value);
		srv.request.knowledge.push_back(item);


		// add agent instance
		srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
		diagnostic_msgs::KeyValue value_add_agent;
		item = rosplan_knowledge_msgs::KnowledgeItem();
		item.knowledge_type = static_cast<int>(Knowledge_type::instance);
		item.instance_type.assign(string("agent"));
		item.instance_name.assign(string("robot") + m_agent_num);
		srv.request.knowledge.push_back(item);

	}

	m_agent_pos_x = (msg->pose.position.x - m_resolution / 2) / m_resolution;
	m_agent_pos_y = m_grid_size - (msg->pose.position.y + m_resolution / 2.0) / m_resolution;
	cout << "x,y: " << m_agent_pos_x << ", " << m_agent_pos_y << endl;

	// add current pose info.
	srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
	item = rosplan_knowledge_msgs::KnowledgeItem();
	diagnostic_msgs::KeyValue value, value_loc;
	item.knowledge_type = static_cast<int>(Knowledge_type::fact);
	item.attribute_name.assign(string("at_location"));
	value.key.assign(string("a"));
	value.value.assign(string("robot") + m_agent_num);
	item.values.push_back(value);

	value_loc.key.assign(string("l"));
	string location = string("f") + to_string(m_agent_pos_y) + string("-") + to_string(m_agent_pos_x) + string("f");
	value_loc.value.assign(location);
	item.values.push_back(value_loc);
	srv.request.knowledge.push_back(item);

	// remove prev pose info. if prev_pose != current_pose
	if (m_agent_pos_x != m_prev_pos_x || m_agent_pos_y != m_prev_pos_y){
		srv.request.update_type.push_back(static_cast<int>(Update_type::remove_knowledge));
		item = rosplan_knowledge_msgs::KnowledgeItem();
		diagnostic_msgs::KeyValue value_prev, value_loc_prev;
		item.knowledge_type = static_cast<int>(Knowledge_type::fact);
		item.attribute_name.assign(string("at_location"));
		value_prev.key.assign(string("a"));
		value_prev.value.assign(string("robot") + m_agent_num);
		item.values.push_back(value_prev);

		value_loc_prev.key.assign(string("l"));
		string prev_location = string("f") + to_string(m_prev_pos_y) + string("-") + to_string(m_prev_pos_x) + string("f");
		value_loc_prev.value.assign(prev_location);
		item.values.push_back(value_loc_prev);
		srv.request.knowledge.push_back(item);
	}



	while (m_client.call(srv) != true){
		ROS_ERROR("KnowledgeUpdaterMultiIndividual::poseCallback: Failed to call service knowledge_update");
		sleep(1);
	}
	ROS_INFO("poseCallback Knowledge Update Service Result: %d", srv.response.success);

	m_prev_pos_x = m_agent_pos_x;
	m_prev_pos_y = m_agent_pos_y;
	return;
}

void KnowledgeUpdaterMultiIndividual::observationCallback(const hiprl_replicate::Obs_multi::ConstPtr &msg){
	ROS_INFO("observationCallback entered");
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
	rosplan_knowledge_msgs::KnowledgeItem item;
	int index = 0;


	for (int agent = 0; agent < m_total_agent_num; agent++){
		for (index = 0; index < msg->observation_list[agent].type_id_list.size(); index++){
			int object_pos_x = msg->observation_list[agent].object_pos_list.at(2*index);
			int object_pos_y = msg->observation_list[agent].object_pos_list.at(2*index+1);

			item = rosplan_knowledge_msgs::KnowledgeItem();
			if (msg->observation_list[agent].type_id_list.at(index) == msg->observation_list[agent].EMPTY){
				// set clear cell true
				srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
				diagnostic_msgs::KeyValue value;
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("clear"));
				value.key.assign(string("v0"));
				string location = string("f") + to_string(object_pos_y) + string("-") + to_string(object_pos_x) + string("f");
				value.value.assign(location);
				item.values.push_back(value);
				srv.request.knowledge.push_back(item);
			} else if (msg->observation_list[agent].type_id_list.at(index) == hiprl_replicate::Obs::WALL){
				// set clear cell false
				srv.request.update_type.push_back(static_cast<int>(Update_type::remove_knowledge));
				diagnostic_msgs::KeyValue value;
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("clear"));
				value.key.assign(string("v0"));
				string location = string("f") + to_string(object_pos_y) + string("-") + to_string(object_pos_x) + string("f");
				value.value.assign(location);
				item.values.push_back(value);
				srv.request.knowledge.push_back(item);
			} else if (msg->observation_list[agent].type_id_list.at(index) == hiprl_replicate::Obs::GOAL){
				// do nothing for the goal
				srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
				diagnostic_msgs::KeyValue value;
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("clear"));
				value.key.assign(string("v0"));
				string location = string("f") + to_string(object_pos_y) + string("-") + to_string(object_pos_x) + string("f");
				value.value.assign(location);
				item.values.push_back(value);
				srv.request.knowledge.push_back(item);
			}

			else if (msg->observation_list[agent].type_id_list.at(index) == hiprl_replicate::Obs::BOX){
				string location = string("f") + to_string(object_pos_y) + string("-") + to_string(object_pos_x) + string("f");

				// set clear false
				srv.request.update_type.push_back(static_cast<int>(Update_type::remove_knowledge));
				diagnostic_msgs::KeyValue value_unclear;
				item = rosplan_knowledge_msgs::KnowledgeItem();
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("clear"));
				value_unclear.key.assign(string("v0"));
				value_unclear.value.assign(location);
				item.values.push_back(value_unclear);
				srv.request.knowledge.push_back(item);
			}

			else if (msg->observation_list[agent].type_id_list.at(index) == hiprl_replicate::Obs::BALL){
				// add instance if unseen object
				if (find(m_seen_obj_id_list.begin(), m_seen_obj_id_list.end(), msg->observation_list[agent].object_id_list.at(index)) != m_seen_obj_id_list.end()){
					// already checked box --> do not add instance
				} else {
					ROS_INFO("ball out of box detected, id: %d", msg->observation_list[agent].object_id_list.at(index));
					m_seen_obj_id_list.push_back(msg->observation_list[agent].object_id_list.at(index));
					// add instance
					srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
					diagnostic_msgs::KeyValue value_add_inst;
					item = rosplan_knowledge_msgs::KnowledgeItem();
					item.knowledge_type = static_cast<int>(Knowledge_type::instance);
					item.instance_type.assign(string("obj"));
					item.instance_name.assign(string("ball") + to_string(msg->observation_list[agent].object_id_list.at(index)));
					srv.request.knowledge.push_back(item);
				}

				// set object_at_location
				srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
				diagnostic_msgs::KeyValue value_obj, value_loc;
				item = rosplan_knowledge_msgs::KnowledgeItem();
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("object_at_location"));
				value_obj.key.assign(string("o"));
				string obj = string("ball") + to_string(msg->observation_list[agent].object_id_list.at(index));
				value_obj.value.assign(obj);
				item.values.push_back(value_obj);

				value_loc.key.assign(string("l"));
				string location = string("f") + to_string(object_pos_y) + string("-") + to_string(object_pos_x) + string("f");
				value_loc.value.assign(location);
				item.values.push_back(value_loc);
				srv.request.knowledge.push_back(item);

				// set out_receptacle
				srv.request.update_type.push_back(static_cast<int>(Update_type::add_knowledge));
				item = rosplan_knowledge_msgs::KnowledgeItem();
				item.knowledge_type = static_cast<int>(Knowledge_type::fact);
				item.attribute_name.assign(string("out_receptacle"));
				item.values.push_back(value_obj);
				srv.request.knowledge.push_back(item);

				// In case the ball is picked up in keep_mode, change mode to bring_mode
				if (object_pos_x == m_prev_pos_x && object_pos_y == m_prev_pos_y && m_keep_mode){
					m_search_mode = false;
					m_keep_mode = false;
					m_bring_mode = true;

					// remove previous goal which is to hold the ball
					srv.request.update_type.push_back(static_cast<int>(Update_type::remove_goal));
					item = rosplan_knowledge_msgs::KnowledgeItem();
					item.knowledge_type = static_cast<int>(Knowledge_type::fact);
					item.attribute_name.assign(string("holds"));
					diagnostic_msgs::KeyValue value_agent;
					value_agent.key.assign(string("a"));
					value_agent.value.assign(string("robot"));
					item.values.push_back(value_agent);
					item.values.push_back(value_obj);
					srv.request.knowledge.push_back(item);


					// add goal to bring the ball to home station
					srv.request.update_type.push_back(static_cast<int>(Update_type::add_goal));
					item = rosplan_knowledge_msgs::KnowledgeItem();
					item.knowledge_type = static_cast<int>(Knowledge_type::fact);
					item.attribute_name.assign(string("object_at_location"));
					diagnostic_msgs::KeyValue value_obj_goal_loc;
					item.values.push_back(value_obj);
					value_obj_goal_loc.key.assign(string("l"));
					value_obj_goal_loc.value.assign(m_goal_pose);
					item.values.push_back(value_obj_goal_loc);
					srv.request.knowledge.push_back(item);

				}
			}

		}

	}


	while (m_client.call(srv) != true){
		ROS_ERROR("Failed to call service knowledge_update");
		sleep(1);
	}
	ROS_INFO("Knowledge Update Service Result: %d", srv.response.success);

	// create dummy goal for knowledge base operation in search mode
	rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv_goal;
	item = rosplan_knowledge_msgs::KnowledgeItem();
	item.knowledge_type = static_cast<int>(Knowledge_type::fact);
	item.attribute_name.assign(string("object_at_location"));
	diagnostic_msgs::KeyValue value_ball, value_loc;
	value_ball.key.assign(string("o"));
	// in case the ball is unique
	value_ball.value.assign(string("ball0"));
	item.values.push_back(value_ball);
	value_loc.key.assign(string("l"));
	value_loc.value.assign(string("f0-0f"));
	item.values.push_back(value_loc);
	srv_goal.request.knowledge.push_back(item);
	ROS_INFO("m_search_mode: %d, m_keep_mode: %d, m_bring_mode: %d", m_search_mode, m_keep_mode, m_bring_mode);
	if (m_seen_box_id_list.size() == m_checked_box_id_list.size() && m_search_mode){
		srv_goal.request.update_type.push_back(static_cast<int>(Update_type::add_goal));
	} else {
		srv_goal.request.update_type.push_back(static_cast<int>(Update_type::remove_goal));
	}
	if (m_client.call(srv_goal) != true){
		ROS_ERROR("Failed to call service knowledge_update in goal mode update procedure");
	}
	return;
}
