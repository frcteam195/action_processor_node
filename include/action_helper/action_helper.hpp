#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "action_processor_node/Action.h"
#include "action_processor_node/ActionList.h"
#include "action_processor_node/ActionListParallel.h"
#include "action_processor_node/ActionReq.h"
#include "action_processor_node/ActionUpdate.h"
#include "action_processor_node/CancelOverall.h"
#include "action_processor_node/OverallActions.h"

class ActionHelper{
public:

    enum class ACTION_STATUS
    {
        EXECUTING = 1,
        COMPLETE = 0,
        FAILED = -1
    };



    ActionHelper(ros::NodeHandle* node);
    ~ActionHelper();


    void pending_action_callback( const action_processor_node::ActionList& msg );
    void executing_action_callback( const action_processor_node::ActionList& msg );
    void step();

    bool req_single_action      ( std::string category_name, std::string action_id );
    bool req_series_action      ( std::string category_name, std::vector<std::string> actions );
    bool req_many_series_action ( std::string category_name, std::vector<std::vector<std::string>> actions );

    bool check_action( std::string action_id );
    bool update_action( std::string action_id, ACTION_STATUS stat);

    bool req_cancel_category( std::string category );

    ros::ServiceClient add_action_client;
    ros::ServiceClient update_action_client;
    ros::ServiceClient cancel_overall_client;

    ros::Subscriber pending_action_sub;
    ros::Subscriber executing_action_sub;

    action_processor_node::ActionList pending_actions;
    action_processor_node::ActionList executing_actions;
};
