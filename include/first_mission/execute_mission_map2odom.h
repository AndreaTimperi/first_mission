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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/buffer.h>
#include <tf2/transform_listener.h>

tf2_ros::Buffer buffer_;
tf2_ros::TransformListener listener_;

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh) : listener_(buffer_)


using namespace std;
ros::ServiceClient image_saver_depth_front, image_saver_depth_rear, image_saver_depth_right, image_saver_depth_left;
std_srvs::Empty srv_click_front, srv_click_rear, srv_click_right, srv_click_left;
XmlRpc::XmlRpcValue environment_list, environment_list2, environment_list_push;
int cnt = 0;
state_machine_msgs::StateWithInitialState navigation_goal_msg;
bool moving = false;
ros::Publisher navigation_goal_pub;
ros::Subscriber sub_travel;

void readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg);

void publish_navgoal(std::string name);

int main( int argc, char** argv );