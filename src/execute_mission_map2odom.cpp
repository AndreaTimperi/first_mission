#include "first_mission/execute_mission_map2odom.h"


// Call back function to read the path planning stauts, set the moving flag and call the publish function
void readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg)
{
    if(msg.status == 1 || cnt == 0)
    {
        // ROS_INFO("Moving to the GOAL");
        moving = true;
    }
    else
    {   
        moving = false;
        ROS_INFO("Arrived");
        ros::Duration(1.5).sleep(); 
        // It works if an "image_view image_saver" node is running with the camera_path and the png file name declared
        if(image_saver_depth_front.call(srv_click_front))
        if(image_saver_depth_rear.call(srv_click_rear))
        if(image_saver_depth_right.call(srv_click_right))
        if(image_saver_depth_left.call(srv_click_left))
        ros::Duration(1.5).sleep();
            ROS_INFO("Image taken");
            
    }
    if (cnt == 0)
    {
        cnt ++;
    }
}

// function use to update the Navigation Goal name and publish on the right topic and GoTo that NavGoal
void publish_navgoal(std::string name)
{
    navigation_goal_msg.state.name = "GoTo" + name;
    navigation_goal_msg.state.type = "navigation_behavior_plugins::ReactiveNavigation";
    navigation_goal_msg.state.settings = "- name: navigation_goal\n  type: NavigationGoal\n  value: " + name;
    vector<string> empty_string_vect = {};
    navigation_goal_msg.initial_state.names = empty_string_vect;
    navigation_goal_pub.publish(navigation_goal_msg);
    // cout<<"name"<<endl;
    // cout << name.at(navgoal_num) << endl;
}

int main( int argc, char** argv )
{
    // initialize ROS
    ros::init(argc, argv, "first_mission_map2odom_node");

    // create a Transform Listener to pass from odom to map
    tf::TransformListener listener;

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

    // Record transform value (rot and trasl) between odom and map
    tf::StampedTransform transform;
        try{
         listener.lookupTransform("/map", "/odom",  
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

    // IDEA DI FARE UN CICLO FOR, dove preso un GetParam - cin che settiamo in f(x) dell'area da esplorare, un Param della granularità, aggiorna le pos in modo ciclico
    
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < 4; ++i)
        { 
            environment_list_push["label"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["name"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["pose"]["header"]["frame_id"] = "map";
            environment_list_push["pose"]["pose"]["orientation"]["w"] = 1.0 - transform.getRotation().w() ;
            environment_list_push["pose"]["pose"]["orientation"]["x"] = 0.0 - transform.getRotation().x();
            environment_list_push["pose"]["pose"]["orientation"]["y"] = 0.0 - transform.getRotation().y();
            environment_list_push["pose"]["pose"]["orientation"]["z"] = 0.0 - transform.getRotation().z();
            environment_list_push["pose"]["pose"]["position"]["x"] = transform.getOrigin().x() + 1.0 + i;
            environment_list_push["pose"]["pose"]["position"]["y"] = transform.getOrigin().y() + 0.0 + j;
            environment_list_push["pose"]["pose"]["position"]["z"] = transform.getOrigin().z() + 0.2;
            environment_list_push["tolerance"]["rotation"] = 0.1;
            environment_list_push["tolerance"]["translation"] = 0.05;
            environment_list_push["type"]= "navigation_goal";

            v.push_back(environment_list_push);
            name.push_back(environment_list_push["label"]);
        }
    }

    // From the vector to the XmlRpcValue again to be aligned with the rosparam type
    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
    }

    nh.setParam("/environment/objects", environment_list2);

    image_saver_depth_front = nh.serviceClient<std_srvs::Empty>("/image_saver_front/save");
    image_saver_depth_rear = nh.serviceClient<std_srvs::Empty>("/image_saver_rear/save");
    image_saver_depth_right = nh.serviceClient<std_srvs::Empty>("/image_saver_right/save");
    image_saver_depth_left = nh.serviceClient<std_srvs::Empty>("/image_saver_left/save");

    navigation_goal_pub = nh.advertise<state_machine_msgs::StateWithInitialState>("/behavior_engine/execute_state/start_state", 1000);

    sub_travel = nh.subscribe("/path_planning_and_following/path_follower/status", 1, readCallback);

    sleep(1);
    ros::Rate rate(400);

    int navgoal_num = 0;
    sleep(1);
    
    // Main loop to ispect iteratively the navigation goal grid
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