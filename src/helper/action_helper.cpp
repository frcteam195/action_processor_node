
#include "helper/action_helper.hpp"

#include <../../../lib_uuid_node/include/lib_uuid_node.hpp>


ActionHelper::ActionHelper(ros::NodeHandle* node)
{
    add_action_client = node->serviceClient<action_processor_node::ActionReq>("/add_action_request");
    update_action_client = node->serviceClient<action_processor_node::ActionUpdate>("/update_action_request");
    cancel_overall_client = node->serviceClient<action_processor_node::CancelOverall>("/cancel_overall_action");
}


ActionHelper::~ActionHelper()
{

}

bool ActionHelper::req_single_action( std::string category_name,
                                      std::string action_id )
{
    action_processor_node::ActionListParallel parallel;
    parallel.parallel_action_id.uuid = libuuid::create_uuid();
    parallel.parallel_action_id.name = category_name;

    action_processor_node::ActionList series_list;

    action_processor_node::Action action;
    action.action_id = action_id;
    action.uuid = libuuid::create_uuid();
    action.timeout = ros::Time(10); // 10s

    series_list.actions.push_back( action );
    parallel.action_list.push_back( series_list );

    action_processor_node::ActionReq req;
    req.request.parallel_list = parallel;
    add_action_client.call( req );

    return true;
}

bool ActionHelper::req_series_action ( std::string category_name,
                                       std::vector<std::string> actions)
{
    action_processor_node::ActionListParallel parallel;
    parallel.parallel_action_id.uuid = libuuid::create_uuid();
    parallel.parallel_action_id.name = category_name;

    action_processor_node::ActionList series_list;

    for( int i = 0; (size_t)i < actions.size(); i++ )
    {
        action_processor_node::Action action;
        action.action_id = actions[i];
        action.uuid = libuuid::create_uuid();
        action.timeout = ros::Time(10); // 10s
        series_list.actions.push_back( action );
    }
    parallel.action_list.push_back( series_list );

    action_processor_node::ActionReq req;
    req.request.parallel_list = parallel;
    add_action_client.call( req );

    return true;
}

bool ActionHelper::req_many_series_action ( std::string category_name,
                                            std::vector<std::vector<std::string>> action_list )
{
    action_processor_node::ActionListParallel parallel;
    parallel.parallel_action_id.uuid = libuuid::create_uuid();
    parallel.parallel_action_id.name = category_name;

    action_processor_node::ActionList series_list;

    for( int p_i = 0; (size_t)p_i < action_list.size(); p_i++ )
    {
        for( int a_i = 0; (size_t)a_i < action_list[p_i].size(); a_i++ )
        {
            action_processor_node::Action action;
            action.action_id = action_list[p_i][a_i];
            action.uuid = libuuid::create_uuid();
            action.timeout = ros::Time(10); // 10s
            series_list.actions.push_back( action );
        }
        parallel.action_list.push_back( series_list );
    }

    action_processor_node::ActionReq req;
    req.request.parallel_list = parallel;
    add_action_client.call( req );

    return true;
}

bool ActionHelper::req_cancel_category( std::string category )
{
    action_processor_node::CancelOverall req;

    req.request.parallel_action_id.name = category;

    cancel_overall_client.call(req);

    return true;
}
