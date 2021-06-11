#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
import numpy as np
query = []

def call_service():
    print("Waiting for service")
    rospy.wait_for_service('/rosplan_knowledge_base/query_state')
    try:
        print("Calling Service")
        query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/query_state', KnowledgeQueryService)
        resp1 = query_proxy(query)
        result = np.array(resp1.results, dtype=bool)
        print("Response is:")
        print(np.reshape(result, (10,10)))
    except (rospy.ServiceException, e):
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    for j in range(10):
        for i in range(10):
            # QUERY 1 (robot_at kenny fi-jf)
            query1 = KnowledgeItem()
            query1.knowledge_type = KnowledgeItem.FACT
            query1.attribute_name = "at_location"
            curr_location = 'f' + str(i) + '-' + str(j) + 'f'

            query1.values.append(diagnostic_msgs.msg.KeyValue("a", "robot"))
            query1.values.append(diagnostic_msgs.msg.KeyValue("l", curr_location))
            query.append(query1)
        
    call_service()
    sys.exit(1)