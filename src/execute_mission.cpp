#include "ros/service_client.h"
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>
#include <array>
#include "ros/ros.h"
#include "navigation_manager_msgs/LocalPlannerStatus.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"
#include "state_machine_msgs/StateWithInitialState.h"
#include "state_machine_msgs/NestedStateName.h"


using namespace std;
ros::ServiceClient client;
std_srvs::Empty srv;
XmlRpc::XmlRpcValue environment_list, environment_list2, environment_list_push;
int tyr_dim, tyr_dim2;
state_machine_msgs::StateWithInitialState navigation_goal_msg;
bool moving = false;
ros::Publisher navigation_goal_pub;

void readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg)
{
    if(msg.status == 1)
    {
        // ROS_INFO("Moving to the GOAL");
        moving = true;
    }
    else
    {   
        moving = false;
        ROS_INFO("Arrived");

        // It works if an image_view image_saver node is running with the camera_path and the png file name declared
        if(client.call(srv))
            ROS_INFO("Image taken");
    }
}

void publish_navgoal(std::string name)
{
    navigation_goal_msg.state.name = "GoTo" + name;
    navigation_goal_msg.state.type = "navigation_behavior_plugins::ReactiveNavigation";
    navigation_goal_msg.state.settings = "- name: navigation_goal\n  type: NavigationGoal\n  value: " + name;
    vector<string> empty_string_vect = {};
    navigation_goal_msg.initial_state.names = empty_string_vect;
    navigation_goal_pub.publish(navigation_goal_msg);
}

int main( int argc, char** argv )
{
    // initialize ROS
    ros::init(argc, argv, "first_mission_node");

    // create a node
    ros::NodeHandle nh;
    nh.getParam("/environment/objects", environment_list);

    vector<string> name;
    environment_list_push = environment_list[0];
    auto a = environment_list[0];
    std::vector<decltype(a)> v;
    for (int i = 0; i < environment_list.size(); i++)
    {
            v.push_back(environment_list[i]);
            // cout << v[i] << " ";
    }

    // IDEA DI FARE UN CICLO FOR, dove preso un GetParam - cin che settiamo in f(x) dell'area da esplorare, un Param della granularità, aggiorna le pos in modo ciclico
    
    for (int j = 0; j < 5; ++j)
    {
        for (int i = 0; i < 5; ++i)
        { 
            environment_list_push["label"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["name"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["pose"]["header"]["frame_id"] = "odom";
            environment_list_push["pose"]["pose"]["orientation"]["w"] = 1.0;
            environment_list_push["pose"]["pose"]["orientation"]["x"] = 0.0;
            environment_list_push["pose"]["pose"]["orientation"]["y"] = 0.0;
            environment_list_push["pose"]["pose"]["orientation"]["z"] = 0.0;
            environment_list_push["pose"]["pose"]["position"]["x"] = 1.0 + i;
            environment_list_push["pose"]["pose"]["position"]["y"] = 0.0 + j;
            environment_list_push["pose"]["pose"]["position"]["z"] = 0.531;
            environment_list_push["tolerance"]["rotation"] = 0.1;
            environment_list_push["tolerance"]["translation"] = 0.05;
            environment_list_push["type"]= "navigation_goal";

            v.push_back(environment_list_push);
            name.push_back(environment_list_push["label"]);
        }
    }

    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
    }

    nh.setParam("/environment/objects", environment_list2);

    // cout<<"\nHow is made the environment_list2 pushed into SetParam  "<<endl;
    // cout<<environment_list2<<endl;

    client = nh.serviceClient<std_srvs::Empty>("/image_saver/save");

    navigation_goal_pub = nh.advertise<state_machine_msgs::StateWithInitialState>("/behavior_engine/execute_state/start_state", 1000);

    ros::Subscriber sub_travel = nh.subscribe("/path_planning_and_following/path_follower/status", 1, readCallback);

    sleep(1);
    ros::Rate rate(400);

    int navgoal_num = 0;
    sleep(1);
    while(ros::ok())
    {
    

    if(!moving && navgoal_num < name.size())
    {
        publish_navgoal(name.at(navgoal_num));
        moving = true;
        navgoal_num ++;
    }
    ros::spinOnce();
    rate.sleep();
  
    }

    return 0;
}