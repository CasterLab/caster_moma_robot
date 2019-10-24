#include <ros/ros.h>
#include <string>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <aruco_msgs/MarkerArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <caster_man_app/PickGift.h>

double body_height;
int get_new_target;

void SetArmPose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double>& joint_value) {
  move_group.setJointValueTarget(joint_value);
  move_group.move();
}

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO_STREAM("Move base done callback");
}

void MoveToGoal(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &move_base_client, geometry_msgs::Pose pose) {
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "map";
  mb_goal.target_pose.pose = pose;

  move_base_client.sendGoal(mb_goal,
            boost::bind(&MovebaseDoneCallback, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&MovebaseFeedbackCallback, _1));
}

void UpdateObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "object";
  collision_objects[0].id = "marker_88";

  // define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.01;
  collision_objects[0].primitives[0].dimensions[1] = 0.1;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  // define the pose of the object
  collision_objects[0].primitive_poses.resize(1);

  // add object to planning scene
  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void GetGoalPose(std::string goal_name, ros::NodeHandle &private_nh, geometry_msgs::Pose &pose) {
  std::vector<float> pose_vector(3), orientation_vector(4);

  private_nh.getParam("target_goal/"+goal_name+"/pose", pose_vector);
  private_nh.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

  pose.position.x = pose_vector[0];
  pose.position.y = pose_vector[1];
  pose.position.z = pose_vector[2];
  pose.orientation.x = orientation_vector[0];
  pose.orientation.y = orientation_vector[1];
  pose.orientation.z = orientation_vector[2];
  pose.orientation.w = orientation_vector[3];
}

bool PickGiftCB(caster_man_app::PickGift::Request &req, caster_man_app::PickGift::Response &res) {
  if(get_new_target < 0) {
    get_new_target = atoi(req.id.c_str());
    res.result = 0;
  } else {
    res.result = -1;
  }

  ROS_INFO("get request: %d, accepted: %d", get_new_target, res.result);
  return true;
}

void JointStatesCB(const sensor_msgs::JointState::ConstPtr& msg) {
  for(uint8_t i=0; i<msg->name.size(); i++) {
    if(msg->name[i] == "caster_body_connected_joint") {
      body_height = msg->position[i];
      // ROS_INFO("Get height %lf", body_height);
      break;
    }
  }
}

bool SetHeight(ros::Publisher &publisher, double height) {
  if(height < 0.0) {
    height = 0;
  } else if (height > 0.35) {
    height = 0.35;
  }

  ROS_INFO("%lf, %lf", height, body_height);

  std_msgs::Float64 msg;
  msg.data = height;
  publisher.publish(msg);

  while(body_height != height) {
    ros::WallDuration(0.2).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_man_app_node");
  ros::NodeHandle nh, private_nh("~");

  // get target pose
  geometry_msgs::Pose pick_pose, place_pose, standby_pose;
  GetGoalPose("pick", private_nh, pick_pose);
  GetGoalPose("place", private_nh, place_pose);
  GetGoalPose("standby", private_nh, standby_pose);

  // get arm standby pose
  std::vector<double> arm_standby_pose(7);
  private_nh.getParam("arm_pose/standby", arm_standby_pose);

  // get arm box-pick pose
  std::vector<std::vector<double>> arm_box_pose(10, std::vector<double>(7));
  private_nh.getParam("arm_pose/pos_0", arm_box_pose[0]);
  private_nh.getParam("arm_pose/box_1", arm_box_pose[1]);
  private_nh.getParam("arm_pose/box_2", arm_box_pose[2]);
  private_nh.getParam("arm_pose/box_3", arm_box_pose[3]);
  private_nh.getParam("arm_pose/box_4", arm_box_pose[4]);
  private_nh.getParam("arm_pose/box_5", arm_box_pose[5]);
  private_nh.getParam("arm_pose/box_6", arm_box_pose[6]);

  // set up spinner
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  arm_group.setPlanningTime(45.0);

  // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));
  ros::Subscriber joint_states_sub = nh.subscribe("joint_states", 1000, JointStatesCB);
  ros::Publisher body_pub = nh.advertise<std_msgs::Float64>("caster/body_controller/command", 1000);

  // // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();

  ROS_INFO("Set Height 0.16");
  SetHeight(body_pub, 0.16);

  ROS_INFO("Moveto standby goal");
  // MoveToGoal(move_base_client, standby_pose);

  ROS_INFO("Set arm to standby pose");
  SetArmPose(arm_group, arm_standby_pose);

  ROS_INFO("Open Gripper");
  std::vector<double> gripper_pose(3);
  gripper_pose[0] = 0.02;
  gripper_pose[1] = 0.02;
  gripper_pose[2] = 0.02;
  gripper_group.setJointValueTarget(gripper_pose);
  gripper_group.move();

  ROS_INFO("Start service...");
  ros::ServiceServer pickup_service = nh.advertiseService("ros_sharp/PickGift", PickGiftCB);

  geometry_msgs::PoseStamped target;
  get_new_target = -1;
  double r_height;
  while(ros::ok()) {
    if(get_new_target < 1 || get_new_target > 6) {
      ros::WallDuration(1.0).sleep();
      continue;
    }

    ROS_INFO("Get new target %d", get_new_target);

    ROS_INFO("Move to pick goal");
    // MoveToGoal(move_base_client, pick_pose);

    if(get_new_target < 3) {
      r_height = 0.34;
    } else if(get_new_target < 5) {
      r_height = 0.16;
    } else {
      r_height = 0.01;
    }
    ROS_INFO("Set height %lf", r_height);
    // SetHeight(body_pub, r_height);

    // set arm to target box watch pose
    ROS_INFO("Set arm to watch pose");
    // SetArmPose(arm_group, arm_box_pose[get_new_target]);

    ROS_INFO("Update object");
    // UpdateObject(planning_scene_interface);

    ROS_INFO("Set arm to pre-pick pose");
    // target.header.stamp = ros::Time::now();
    // target.header.frame_id = "object";
    // target.pose.position.y = 0.02;
    // target.pose.position.z = -0.10;
    // arm_group.setPoseTarget(target);
    // arm_group.move();

    ROS_INFO("Open Gripper");
    // gripper_pose[0] = 0.02;
    // gripper_pose[1] = 0.02;
    // gripper_pose[2] = 0.02;
    // gripper_group.setJointValueTarget(gripper_pose);
    // gripper_group.move();

    ROS_INFO("Set arm to pick pose");
    // target.header.stamp = ros::Time::now();
    // target.header.frame_id = "object";
    // target.pose.position.y = 0.02;
    // target.pose.position.z = -0.03;
    // arm_group.setPoseTarget(target);
    // arm_group.move();

    ROS_INFO("Close Gripper");
    // gripper_pose[0] = 1.2;
    // gripper_pose[1] = 1.2;
    // gripper_pose[2] = 1.2;
    // gripper_group.setJointValueTarget(gripper_pose);
    // gripper_group.move();

    ROS_INFO("Set arm to retreat-pick pose");
    // target.header.stamp = ros::Time::now();
    // target.header.frame_id = "object";
    // target.pose.position.y = 0.10;
    // target.pose.position.z = -0.20;
    // arm_group.setPoseTarget(target);
    // arm_group.move();

    ROS_INFO("Hold the box")
    // SetArmPose(arm_group, arm_standby_pose);

    ROS_INFO("Set Height 0.16");
    // SetHeight(body_pub, 0.16);

    ROS_INFO("Move to table")
    // MoveToGoal(move_base_client, place_pose);

    ROS_INFO("Set arm to pre-place pose");
    // SetArmPose(arm_group, arm_standby_pose);

    ROS_INFO("Set arm to pre-place pose");
    // SetArmPose(arm_group, arm_standby_pose);

    ROS_INFO("Open Gripper");
    // gripper_pose[0] = 0.02;
    // gripper_pose[1] = 0.02;
    // gripper_pose[2] = 0.02;
    // gripper_group.setJointValueTarget(gripper_pose);
    // gripper_group.move();

    ROS_INFO("Set arm to home pose");
    // SetArmPose(arm_group, arm_standby_pose);

    ROS_INFO("Back to standby pose");
    // MoveToGoal(move_base_client, standby_pose);

    ROS_INFO("Reset flag");
    get_new_target = -1;

    ROS_INFO("All finish");
  }

  ros::waitForShutdown();
  return 0;
}