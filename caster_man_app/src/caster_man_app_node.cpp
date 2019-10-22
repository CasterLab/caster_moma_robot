#include <ros/ros.h>

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

void openGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2n6s300_joint_finger_1";
  posture.joint_names[1] = "j2n6s300_joint_finger_2";
  posture.joint_names[2] = "j2n6s300_joint_finger_3";

  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 0.1;
  posture.points[0].positions[1] = 0.1;
  posture.points[0].positions[2] = 0.1;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2n6s300_joint_finger_1";
  posture.joint_names[1] = "j2n6s300_joint_finger_2";
  posture.joint_names[2] = "j2n6s300_joint_finger_3";

  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 1.1;
  posture.points[0].positions[1] = 1.1;
  posture.points[0].positions[2] = 1.1;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "object";

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.0;
  grasps[0].grasp_pose.pose.position.y = 0.03;
  grasps[0].grasp_pose.pose.position.z = -0.02;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "j2n6s300_end_effector";
  grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.16;
  grasps[0].pre_grasp_approach.desired_distance = 0.18;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "j2n6s300_end_effector";
  grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
  grasps[0].post_grasp_retreat.direction.vector.y = 0.7;
  grasps[0].post_grasp_retreat.min_distance = 0.12;
  grasps[0].post_grasp_retreat.desired_distance = 0.15;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.pick("marker_88", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "j2n6s300_link_base";
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI/2.0, 0.00, -M_PI/2.0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = -0.519f;
  place_location[0].place_pose.pose.position.y = 0.336f;
  place_location[0].place_pose.pose.position.z = 0.515f;

  place_location[0].pre_place_approach.direction.header.frame_id = "j2n6s300_end_effector";
  place_location[0].pre_place_approach.direction.vector.y = -0.6f;
  place_location[0].pre_place_approach.direction.vector.z = 1.0f;
  place_location[0].pre_place_approach.min_distance = 0.08f;
  place_location[0].pre_place_approach.desired_distance = 0.10f;

  place_location[0].post_place_retreat.direction.header.frame_id = "j2n6s300_end_effector";
  place_location[0].post_place_retreat.direction.vector.z = -1.0f;
  place_location[0].post_place_retreat.min_distance = 0.1f;
  place_location[0].post_place_retreat.desired_distance = 0.25f;

  openGripper(place_location[0].post_place_posture);

  group.place("marker_88", place_location);
}

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO_STREAM("Move base done callback");
}

void MoveToGoal(geometry_msgs::Pose pose, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &move_base_client) {
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_man_app_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // set up spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // get target pose
  geometry_msgs::Pose pick_pose, place_pose, standby_pose;
  GetGoalPose("pick", private_nh, pick_pose);
  GetGoalPose("place", private_nh, place_pose);
  GetGoalPose("standby", private_nh, standby_pose);

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  // arm_group.setPlanningTime(45.0);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));

  // // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();

  // move to standby goal
  MoveToGoal(standby_pose, move_base_client);

  while(ros::ok()) {
    if(get_new_target == false) {
      ros::WallDuration(1.0).sleep();
      continue;
    }

    // move to pick pose
    MoveToGoal(pick_pose, move_base_client);

    // set arm to target box watch pose
    SetArmPose(target_box_watch_pose[target_box]);

    // UpdateObject(planning_scene_interface)

    // pick up box
    // pick(arm_group);

    ros::WallDuration(1.0).sleep();

    // move to place pose
    MoveToGoal(place_pose, move_base_client);

    // place box on table
    // place(arm_group);

    // set arm to home pose
    SetArmPose(arm_home_pose);

    ros::WallDuration(1.0).sleep();

    // back to standby pose
    MoveToGoal(standby_pose, move_base_client);
  }

  // UpdateObject(planning_scene_interface);

  // ros::WallDuration(2.0).sleep();

  // pick(arm_group);

  // ros::WallDuration(1.0).sleep();

  // place(arm_group);

  ros::waitForShutdown();
  return 0;
}