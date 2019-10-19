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
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2n6s300_joint_finger_1";
  posture.joint_names[1] = "j2n6s300_joint_finger_2";
  posture.joint_names[2] = "j2n6s300_joint_finger_3";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 0.2;
  posture.points[0].positions[1] = 0.2;
  posture.points[0].positions[2] = 0.2;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2n6s300_joint_finger_1";
  posture.joint_names[1] = "j2n6s300_joint_finger_2";
  posture.joint_names[2] = "j2n6s300_joint_finger_3";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 1.20;
  posture.points[0].positions[1] = 1.20;
  posture.points[0].positions[2] = 1.20;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  tfBuffer.canTransform("base_link", "object", ros::Time(0), ros::Duration(4.0));
  transformStamped = tfBuffer.lookupTransform("base_link", "object", ros::Time(0));

  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  grasps[0].grasp_pose.header.frame_id = "base_link";
  // tf2::Quaternion orientation;
  // orientation.setRPY(0.0, 0.0, 0.0);
  grasps[0].grasp_pose.pose.orientation = transformStamped.transform.rotation;
  grasps[0].grasp_pose.pose.position.x = transformStamped.transform.translation.x;
  grasps[0].grasp_pose.pose.position.y = transformStamped.transform.translation.y;
  grasps[0].grasp_pose.pose.position.z = transformStamped.transform.translation.z;

  // orientation.setRPY(0, 0, 0);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  // grasps[0].grasp_pose.pose.position.x = 0;
  // grasps[0].grasp_pose.pose.position.y = 0;
  // grasps[0].grasp_pose.pose.position.z = 0;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  // move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group) {
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "root";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "root";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "root";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  // group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  // // BEGIN_SUB_TUTORIAL table1
  // //
  // // Creating Environment
  // // ^^^^^^^^^^^^^^^^^^^^
  // // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // // Add the first table where the cube will originally be kept.
  // collision_objects[0].id = "table1";
  // collision_objects[0].header.frame_id = "base_link";

  // /* Define the primitive and its dimensions. */
  // collision_objects[0].primitives.resize(1);
  // collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  // collision_objects[0].primitives[0].dimensions.resize(3);
  // collision_objects[0].primitives[0].dimensions[0] = 0.2;
  // collision_objects[0].primitives[0].dimensions[1] = 0.4;
  // collision_objects[0].primitives[0].dimensions[2] = 0.4;

  // /* Define the pose of the table. */
  // collision_objects[0].primitive_poses.resize(1);
  // collision_objects[0].primitive_poses[0].position.x = 0.5;
  // collision_objects[0].primitive_poses[0].position.y = 0;
  // collision_objects[0].primitive_poses[0].position.z = 0.2;
  // // END_SUB_TUTORIAL

  // collision_objects[0].operation = collision_objects[0].ADD;

  // // BEGIN_SUB_TUTORIAL table2
  // // Add the second table where we will be placing the cube.
  // collision_objects[1].id = "table2";
  // collision_objects[1].header.frame_id = "base_link";

  // /* Define the primitive and its dimensions. */
  // collision_objects[1].primitives.resize(1);
  // collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  // collision_objects[1].primitives[0].dimensions.resize(3);
  // collision_objects[1].primitives[0].dimensions[0] = 0.4;
  // collision_objects[1].primitives[0].dimensions[1] = 0.2;
  // collision_objects[1].primitives[0].dimensions[2] = 0.4;

  // /* Define the pose of the table. */
  // collision_objects[1].primitive_poses.resize(1);
  // collision_objects[1].primitive_poses[0].position.x = 0;
  // collision_objects[1].primitive_poses[0].position.y = 0.5;
  // collision_objects[1].primitive_poses[0].position.z = 0.2;
  // // END_SUB_TUTORIAL

  // collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO_STREAM("Move base done callback");
}

void moveto_standby() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "map";
  // mb_goal.target_pose.pose = dock_ready_pose_;

  move_base_client.sendGoal(mb_goal,
            boost::bind(&MovebaseDoneCallback, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&MovebaseFeedbackCallback, _1));
}

void moveto_desk() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "map";
  // mb_goal.target_pose.pose = dock_ready_pose_;

  move_base_client.sendGoal(mb_goal,
            boost::bind(&MovebaseDoneCallback, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&MovebaseFeedbackCallback, _1));
}

void MarkerPoseCallback(const aruco_msgs::MarkerArray::ConstPtr& msg, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped1, transformStamped2;
  
  transformStamped1.header.stamp = ros::Time::now();
  transformStamped1.header.frame_id = "camera_color_optical_frame";
  transformStamped1.child_frame_id = "marker";
  transformStamped1.transform.translation.x = msg->markers[0].pose.pose.position.x;
  transformStamped1.transform.translation.y = msg->markers[0].pose.pose.position.y;
  transformStamped1.transform.translation.z = msg->markers[0].pose.pose.position.z;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);
  transformStamped1.transform.rotation.x = msg->markers[0].pose.pose.orientation.x;
  transformStamped1.transform.rotation.y = msg->markers[0].pose.pose.orientation.y;
  transformStamped1.transform.rotation.z = msg->markers[0].pose.pose.orientation.z;
  transformStamped1.transform.rotation.w = msg->markers[0].pose.pose.orientation.w;
  br.sendTransform(transformStamped1);

  transformStamped2.header.stamp = ros::Time::now();
  transformStamped2.header.frame_id = "marker";
  transformStamped2.child_frame_id = "object";
  transformStamped2.transform.translation.x = 0;
  transformStamped2.transform.translation.y = -0.036;
  transformStamped2.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);
  transformStamped2.transform.rotation.x = q.x();
  transformStamped2.transform.rotation.y = q.y();
  transformStamped2.transform.rotation.z = q.z();
  transformStamped2.transform.rotation.w = q.w();
  br.sendTransform(transformStamped2);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "object";
  collision_objects[0].id = "marker_46";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.072;
  collision_objects[0].primitives[0].dimensions[1] = 0.1;
  collision_objects[0].primitives[0].dimensions[2] = 0.072;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  // collision_objects[0].primitive_poses[0] = msg->markers[0].pose.pose;
  // collision_objects[0].primitive_poses[0].position = msg->pose.position;
  // collision_objects[0].primitive_poses[0].position.x = msg->pose.position.x;
  // collision_objects[0].primitive_poses[0].position.y = msg->pose.position.y;
  // collision_objects[0].primitive_poses[0].position.z = msg->pose.position.z;

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);

  // ROS_INFO("get Marker %f", msg->pose.position.x);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_man_app_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  // moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  arm_group.setPlanningTime(45.0);

  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));

  // addCollisionObjects(planning_scene_interface);

  // // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(arm_group);

  // ros::WallDuration(1.0).sleep();

  // place(arm_group);

  ros::waitForShutdown();
  return 0;
}