#include <first_mission/execute_mission_base2map.h>


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
    ros::init(argc, argv, "first_mission_base2map_node");

    // create a Transform Samperd to pass from odom to map
    geometry_msgs::TransformStamped transformStamped;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

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
    
    cout << "Set the ispection area" << endl;
    cout << "Number of rows: " << endl;
    cin >> rows;
    cout << "Number of columns: " << endl;
    cin >> columns;

    for (int j = 0; j < columns; ++j)
    {
        for (int i = 0; i < rows; ++i)
        { 
            // Record transform value (rot and trasl) between odom and map
            try{
                tfBuffer.canTransform("map", "base", ros::Time(0), ros::Duration(3.0));
                transformStamped = tfBuffer.lookupTransform("map", "base", ros::Time(0));
            }
            catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            Eigen::Vector3d goal_position_base(1.0 + i, 0.0 + j, 0);
            Eigen::Isometry3d base_to_map = tf2::transformToEigen(transformStamped);

            Eigen::Vector3d goal_position_map = base_to_map*goal_position_base;

            environment_list_push["label"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["name"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["pose"]["header"]["frame_id"] = "map";
            environment_list_push["pose"]["pose"]["orientation"]["w"] = transformStamped.transform.rotation.w;
            environment_list_push["pose"]["pose"]["orientation"]["x"] = transformStamped.transform.rotation.x;
            environment_list_push["pose"]["pose"]["orientation"]["y"] = transformStamped.transform.rotation.y;
            environment_list_push["pose"]["pose"]["orientation"]["z"] = transformStamped.transform.rotation.z;
            environment_list_push["pose"]["pose"]["position"]["x"] = goal_position_map(0);
            environment_list_push["pose"]["pose"]["position"]["y"] = goal_position_map(1);
            environment_list_push["pose"]["pose"]["position"]["z"] = goal_position_map(2); // TODO controllare questo perchè se ci sono pendenza vada bene
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