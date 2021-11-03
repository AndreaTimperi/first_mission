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

void readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg)
{
    if(msg.status == 1)
    {
        ROS_INFO("Moving to the GOAL");
    }
    else
    {   
        // It works if an image_view image_saver node is running with the camera_path and the png file name declared
        if(client.call(srv))
            ROS_INFO("Arrived");
    }
}

// template<class A>
// void info(const A&)
// {
//     typedef typename std::remove_all_extents<A>::type Type;
//     std::cout << "underlying type: " << typeid(Type).name() << '\n';
// }

int main( int argc, char** argv )
{
    // initialize ROS
    ros::init(argc, argv, "first_mission_node");

    // create a node
    ros::NodeHandle nh;
    nh.getParam("/environment/objects", environment_list);

    // Find which is the type of environment_list
    if(environment_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        ROS_INFO( " environment_list -> I AM A STRINGA");
    };
    if(environment_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_INFO( " environment_list -> I AM AN ARRAY");
    };
    if(environment_list.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        ROS_INFO( " environment_list -> I AM A DOUBLE");
    };
    if(environment_list.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_INFO( " environment_list -> I AM A STRUCT");
    };

    // Find which is the type of environment_list elements
    if(environment_list[1].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        ROS_INFO( " environment_list[1] -> I AM A STRINGA");
    };
    if(environment_list[1].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_INFO( " environment_list[1] -> I AM AN ARRAY");
    };
    if(environment_list[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        ROS_INFO( " environment_list[1] -> I AM A DOUBLE");
    };
    if(environment_list[1].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_INFO( " environment_list[1] -> I AM A STRUCT");
    };

    environment_list_push = environment_list[0];
    auto a = environment_list[0];
    std::vector<decltype(a)> v;
    for (int i = 0; i < environment_list.size(); i++)
    {
            v.push_back(environment_list[i]);
            cout << v[i] << " ";
    }
    // cout<<"\nCheck whats inside the struct"<<endl;
    // cout<<environment_list[0]["pose"]["header"]["frame_id"] <<endl;
    // cout<<environment_list_push["pose"]["header"]["frame_id"] <<endl;

    // IDEA DI FARE UN CICLO FOR, dove preso un GetParam che settiamo in f(x) dell'area da esplorare, un Param della granularità, aggiorna le pos in modo ciclico
    
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
        }
    }

    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
    }

    // cout<<environment_list_push["pose"]["header"]["frame_id"] <<endl;

    // environment_list.write(environment_list_push);

    // Check the environment_list dimension
    // tyr_dim = environment_list.size();
    // cout<<"\nCheck how long is environment_list as it is at the beginning"<<endl;
    // cout<<tyr_dim<<endl;

    // Find info about environment_list element
    //info(environment_list[1]);

    // Check if there is a type of environment_list not struct
    // for (int32_t i = 0; i < environment_list.size(); ++i)
    // { 
    //     if (environment_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) 
    //     { 
    //        ROS_ERROR("Params for octomap updater %d not a struct; ignoring.", i); 
    //        continue; 
    //     }
    // }

    // environment_list[tyr_dim] = "[label:PROVAAAAA,name:PROVAAAA,pose:[header:[frame_id:map],pose:[orientation:[w:1,x:0,y:0,z:0],position:[x:-0.7,y:0,z:0.52]],tolerance:[rotation:0.1,translation:0.05]],type:navigation_goal]";
    // cout<<environment_list[tyr_dim]<<endl;

    // tyr_dim = environment_list.size();
    // cout<<tyr_dim<<endl;

    // cout<<environment_list[2]<<endl;
    // std::vector<auto> v(std::begin(environment_list), std::end(environment_list));

    // Converts the XmlRpc::XmlRpcValue into a vector to push_back new NAV GOAL
    // auto a = environment_list[0];
    // std::vector<decltype(a)> v;
    // for (int i = 0; i < environment_list.size(); i++) {
    //         v.push_back(environment_list[i]);
    //         cout << v[i] << " ";
    // }

    // Check the vector create dimension
    // cout<<"\nSecond Check how long is v, vector created, as it is at the beginning"<<endl;
    // cout<<v.size()<<endl;

    // Trying to push back a new NAV GOAL, but as a string, maybe has to be a struct, but how?
    // v.push_back(environment_list_push);
    // Check the vector addded dimension, it should be one more, but has to be also the right type
    // cout<<"\nCheck if I added or not the element, checking the whole dimension"<<endl;
    // cout<<v.size()<<endl;
    
    // Check the element addded
    // cout<<"\nThis is the new element pushed back to the vector"<<endl;
    // cout << v[64] << endl;

    // Fill a new XmlRpc::XmlRpcValue starting from the vector modified
    // for (int i = 0; i < v.size(); i++) {
    //         environment_list2[i] = v[i];
    // }
    
    // tyr_dim2 = environment_list2.size();
    // cout<<"\nNew environment_list2 dimension"<<endl;
    // cout<<tyr_dim2<<endl;
    // cout<<"\nCheck if it hase been inclueded PROVAAAAAa and if it's N6XmlRpc11XmlRpcValueE"<<endl;
    //info(environment_list2[64]);

    // Check if there is a type of environment_list2 not struct
    // for (int32_t i = 0; i < environment_list2.size(); ++i)
    // { 
    //     if (environment_list2[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) 
    //     { 
    //        ROS_ERROR("Params for octomap updater %d not a struct; ignoring.", i); 
    //        continue; 
    //     }
    // }

    // Find which is the type of environment_list2 added elements
    // if(environment_list2[64].getType() == XmlRpc::XmlRpcValue::TypeString)
    // {
    //     ROS_INFO("environment_list2[new] -> I AM A STRINGA");
    // };
    // if(environment_list2[64].getType() == XmlRpc::XmlRpcValue::TypeArray)
    // {
    //     ROS_INFO("environment_list2[new] -> I AM AN ARRAY");
    // };
    // if(environment_list2[64].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    // {
    //     ROS_INFO("environment_list2[new] -> I AM A DOUBLE");
    // };
    // if(environment_list2[64].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    // {
    //     ROS_INFO("environment_list2[new] -> I AM A STRUCT");
    // };

    nh.setParam("/environment/objects", environment_list2);
    // cout<<"\nHow is made the environment_list2 pushed into SetParam  "<<endl;
    // cout<<environment_list2<<endl;

    client = nh.serviceClient<std_srvs::Empty>("/image_saver/save");

    ros::Publisher navigation_goal_pub = nh.advertise<state_machine_msgs::StateWithInitialState>("/behavior_engine/execute_state/start_state", 10);

    ros::Subscriber sub_travel = nh.subscribe("/path_planning_and_following/path_follower/status", 1, readCallback);
    
    ros::Rate rate(1);

    while(ros::ok())
    {
    
    navigation_goal_msg.state.name = "GoToIspectionNavigationGoal00";
    navigation_goal_msg.state.type = "navigation_behavior_plugins::ReactiveNavigation";
    navigation_goal_msg.state.settings = "- name: navigation_goal\n  type: NavigationGoal\n  value: IspectionNavigationGoal00";
    navigation_goal_msg.initial_state.names = '[]';

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}