#include "ros/ros.h"
#include "std_msgs/String.h"

#include "action_processor_node/Action.h"
#include "action_processor_node/ActionList.h"
#include "action_processor_node/ActionListParallel.h"
#include "action_processor_node/ActionReq.h"
#include "action_processor_node/ActionUpdate.h"
#include "action_processor_node/CancelOverall.h"
#include "action_processor_node/OverallActions.h"
#include <uuid_msgs/UniqueID.h>

#include <thread>
#include <string>
#include <mutex>

#define RATE (100)

ros::NodeHandle* node;

// list of all actions
// at this level eachg index is to be executed in series
std::vector<action_processor_node::ActionListParallel> actions;

ros::Publisher executing_action_publisher;
ros::Publisher pending_action_publisher;
ros::Publisher overall_action_publisher;

action_processor_node::Action* find_action( uuid_msgs::UniqueID id )
{
    for( int parallel_i = 0; (size_t)parallel_i < actions.size(); parallel_i++ )
    {
        action_processor_node::ActionListParallel* current_parallel
            = &(actions[parallel_i]);


        for( int i = 0; (size_t)i < current_parallel->action_list.size(); i++ )
        {
            // for every ActionList, take the action that is currently
            // executing and add it to the active list
            // the action that is curretly pending and add it to the pending list

            action_processor_node::ActionList* current_action_list
                = &(current_parallel->action_list[i]);

            for( int action_i = 0; (size_t)action_i < current_action_list->actions.size(); action_i++ )
            {
                action_processor_node::Action* current_action
                    = &(current_action_list->actions[action_i]);

                if( current_action->uuid == id )
                {
                    return current_action;
                }
            }

        }

    }

    return NULL;
}

void step_actions()
{
    for( int parallel_i = 0; (size_t)parallel_i < actions.size(); parallel_i++ )
    {
        // only process the first for now
        if( parallel_i > 0 ){ break; }

        action_processor_node::ActionListParallel* current_parallel
            = &(actions[parallel_i]);

        for( int i = 0; (size_t)i < current_parallel->action_list.size(); i++ )
        {
            action_processor_node::ActionList* current_action_list
                = &(current_parallel->action_list[i]);

            for( int action_i = 0; (size_t)action_i < current_action_list->actions.size(); action_i++ )
            {
                action_processor_node::Action* current_action
                    = &(current_action_list->actions[action_i]);

                // if we are the first action in the list and marked as ADDED, then become pending
                if( action_i == 0 &&
                    current_action->status == action_processor_node::Action::ADDED)
                {
                    current_action->status = action_processor_node::Action::PENDING;
                }

                // if we are not the first action and the action before us is completed or failed
                // mark us as pending
                if( action_i > 0  )
                {
                    action_processor_node::Action* prev_action
                        = &(current_action_list->actions[action_i-1]);

                    if( prev_action->status == action_processor_node::Action::COMPLETE ||
                        prev_action->status == action_processor_node::Action::FAILED )
                    {
                        if( current_action->status == action_processor_node::Action::ADDED )
                        {
                            current_action->status = action_processor_node::Action::PENDING;
                        }
                    }

                }

            }
        }
    }

}

void publish_actions()
{

    action_processor_node::OverallActions output_overall_actions;
    for( int i = 0; (size_t)i < actions.size(); i++ )
    {
        output_overall_actions.parallel_actions.push_back( actions[i].parallel_action_id );
    }

    action_processor_node::ActionList output_executing;
    action_processor_node::ActionList output_pending;

    // we are only ever executing the first index in the actions array
    // so only look at the first index
    if( actions.size() <= 0 )
    {
        executing_action_publisher.publish(output_executing);
        pending_action_publisher.publish(output_pending);
        return;
    }

    action_processor_node::ActionListParallel* current_parallel
        = &(actions[0]);

    for( int i = 0; (size_t)i < current_parallel->action_list.size(); i++ )
    {
        // for every ActionList, take the action that is currently
        // executing and add it to the active list
        // the action that is curretly pending and add it to the pending list

        action_processor_node::ActionList* current_action_list
            = &(current_parallel->action_list[i]);

        for( int action_i = 0; (size_t)action_i < current_action_list->actions.size(); action_i++ )
        {
            action_processor_node::Action* current_action
                = &(current_action_list->actions[action_i]);

            // if the current action pending, add it to the pending list
            // continue, so we don't add future actions to pending list
            if( current_action->status == action_processor_node::Action::PENDING)
            {
                action_processor_node::Action pending_action;
                pending_action = *current_action;
                output_pending.actions.push_back( pending_action );
            }

            // if the current action executing, add it to the executing list
            // continue, so we don't add future actions to pending list
            if( current_action->status == action_processor_node::Action::EXECUTING)
            {
                action_processor_node::Action executing_action;
                executing_action = *current_action;
                output_executing.actions.push_back( executing_action );
            }
        }
    }

    pending_action_publisher.publish(output_pending);
    executing_action_publisher.publish(output_executing);
}


bool add_action_request( action_processor_node::ActionReq::Request& req,
                         action_processor_node::ActionReq::Response& resp )
{

    static int response_counter = 0;

    std::cout << "Got Action Req\n";

    action_processor_node::ActionListParallel insert_parallel;

    // sanitize the input, the input is a list of actionLists to be inserted and run in parallel
    // it will get added the the back of the actions array
    // but we only really care about the action ID, so we will sanitize
    for( int i = 0; (size_t)i < req.parallel_list.action_list.size(); i++)
    {
        action_processor_node::ActionList* current_action_list = &req.parallel_list.action_list[i];
        action_processor_node::ActionList insert_action_list;

        for( int action_i = 0;
             (size_t)action_i < current_action_list->actions.size();
             action_i++ )
        {
            action_processor_node::Action insert_action;
            insert_action.status = action_processor_node::Action::ADDED;
            insert_action.action_id = current_action_list->actions[i].action_id;
            insert_action.uuid = current_action_list->actions[i].uuid;

            insert_action_list.actions.push_back(insert_action);
        }

        insert_parallel.action_list.push_back(insert_action_list);
    }

    resp.id = response_counter++;
    resp.state = action_processor_node::ActionReq::Response::ACCEPTED;
    resp.position_in_queue = actions.size();


    insert_parallel.parallel_action_id = req.parallel_list.parallel_action_id;
    actions.push_back(insert_parallel);

    return true;
}

bool update_action_request( action_processor_node::ActionUpdate::Request& req,
                            action_processor_node::ActionUpdate::Response& resp )
{

    action_processor_node::Action* action = find_action( req.uuid );

    if( action != NULL )
    {
        if( req.status == action_processor_node::Action::EXECUTING ||
            req.status == action_processor_node::Action::COMPLETE  ||
            req.status == action_processor_node::Action::FAILED )
        {
            action->status = req.status;
            resp.status = action_processor_node::ActionUpdate::Response::OK;
            return false;
        }
    }

    resp.status = action_processor_node::ActionUpdate::Response::REJECTED;
    return true;
}

bool cancel_overall_action( action_processor_node::CancelOverall::Request& req,
                            action_processor_node::CancelOverall::Response& resp )
{

    action_processor_node::OverallActions output_overall_actions;
    for( int i = 0; (size_t)i < actions.size(); i++ )
    {
        if( actions[i].parallel_action_id == req.parallel_action_id )
        {
            actions.erase( actions.begin()+1 );
            resp.status = action_processor_node::CancelOverall::Response::OK;
            return true;
        }
    }

    resp.status = action_processor_node::CancelOverall::Response::REJECTED;
    return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_processor_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    executing_action_publisher
        = node->advertise<action_processor_node::ActionList>("executing_actions", 1);

    pending_action_publisher
        = node->advertise<action_processor_node::ActionList>("pending_actions", 1);

    overall_action_publisher
        = node->advertise<action_processor_node::OverallActions>("overall_actions", 1);

    ros::ServiceServer service_add        = n.advertiseService("add_action_request", add_action_request);
    ros::ServiceServer service_update     = n.advertiseService("update_action_request", update_action_request);
    ros::ServiceServer overall_action_req = n.advertiseService("cancel_overall_action", cancel_overall_action);

    while( ros::ok() )
    {

        step_actions();

        publish_actions();

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
