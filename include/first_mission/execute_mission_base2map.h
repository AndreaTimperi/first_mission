#include <ros/ros.h>
#include <ros/service_client.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>
#include <array>
#include <navigation_manager_msgs/LocalPlannerStatus.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <state_machine_msgs/StateWithInitialState.h>
#include <state_machine_msgs/NestedStateName.h>

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>



using namespace std;
ros::ServiceClient image_saver_depth_front, image_saver_depth_rear, image_saver_depth_right, image_saver_depth_left;
std_srvs::Empty srv_click_front, srv_click_rear, srv_click_right, srv_click_left;
XmlRpc::XmlRpcValue environment_list, environment_list2, environment_list_push;
int cnt = 0, rows, columns;
state_machine_msgs::StateWithInitialState navigation_goal_msg;
bool moving = false;
ros::Publisher navigation_goal_pub;
ros::Subscriber sub_travel;

void readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg);

void publish_navgoal(std::string name);

int main( int argc, char** argv );