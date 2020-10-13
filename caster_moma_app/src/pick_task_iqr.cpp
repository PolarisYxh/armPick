#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <caster_moma_app/PickGift.h>
#include <caster_moma_app/PickItemsAction.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/Stop.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pan_tilt_msgs/PanTiltCmd.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef actionlib::ActionServer<caster_moma_app::PickItemsAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> FingerClient;
typedef actionlib::ServerGoalHandle<caster_moma_app::PickItemsAction> GoalHandle;

bool move_goal_flag;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}

class PickTaskAction
{
public:

  ros::NodeHandle nh_;
  Server as_;
  FingerClient fc_;
  
  GoalHandle goal_handle_;
  MoveBaseClient move_base_client_;
  
  std::string action_name_;

  caster_moma_app::PickItemsFeedback feedback_;
  caster_moma_app::PickItemsResult result_;

	double body_height, height_goal_, gripper_position_;
	int get_new_target, obstacle_id_, object_id_, step, box_count_;
	bool find_table_, find_object_, task_active_, get_goal, marker_nav;
	std::map<std::string, std::vector<double>> mm;

	ros::ServiceClient stop_client, start_client;
	ros::Publisher body_pub, head_pub, planning_scene_diff_publisher, base_cmd_pub;
	ros::Subscriber joint_states_sub, marker_pose_sub, table_pose_sub;
	
	// pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
	aruco_msgs::MarkerArray marker_array_msg_;

  std::vector<double> arm_standby_pose_, arm_box_position_, arm_box_size_;
  std::vector<std::vector<double>> arm_box_pose_, pose_vector_, orientation_vector_;

  geometry_msgs::Pose standby_position_, pick_position_, place_position_, table_pose_, object_pose_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group;
  moveit::planning_interface::MoveGroupInterface gripper_group;

  PickTaskAction(std::string name) : 
    as_(nh_, name, boost::bind(&PickTaskAction::ExecuteCb, this, _1), boost::bind(&PickTaskAction::preemptCB, this, _1), false),
		move_base_client_("move_base", true),
		fc_("j2s6s200_driver/fingers_action/finger_positions", true),
    arm_group("arm"),
    gripper_group("gripper")
  {
	  marker_pose_sub = nh_.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, &PickTaskAction::MarkerPoseCallback, this);
	  table_pose_sub = nh_.subscribe<visualization_msgs::Marker>("plane_marker_publisher/marker", 1000, &PickTaskAction::TablePoseCallback, this);
	  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));
	  planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	  base_cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	  joint_states_sub = nh_.subscribe("joint_states", 1000, &PickTaskAction::JointStatesCB, this);
	  body_pub = nh_.advertise<std_msgs::Float64>("caster/body_controller/command", 1000);
	  head_pub = nh_.advertise<pan_tilt_msgs::PanTiltCmd>("pan_tilt_driver_node/pan_tilt_cmd", 1000);
		stop_client = nh_.serviceClient<kinova_msgs::Stop>("/j2s6s200_driver/in/stop");
		start_client = nh_.serviceClient<kinova_msgs::Start>("/j2s6s200_driver/in/start");

	  arm_group.setPlanningTime(45.0);

		GetParam();
    as_.start();
  }

  ~PickTaskAction(void)
  {
  }

  void MarkerPoseCallback(const aruco_msgs::MarkerArrayConstPtr &msg) {
  	// marker_array_msg_ = *msg;
  	for (int i = 0; i < msg->markers.size(); ++i)
  	{
  		if (msg->markers[i].id == 9) {
  			table_pose_ = msg->markers[i].pose.pose;
  			find_table_ = true;
  		}
  		
  		if (msg->markers[i].id == object_id_) {
  			object_pose_ = msg->markers[i].pose.pose;
  			find_object_ = true;
  		}
  	}
  }

  void TablePoseCallback(const visualization_msgs::MarkerConstPtr &msg) {
  	// marker_array_msg_ = *msg;

		geometry_msgs::TransformStamped marker_transform, object_transform;

		table_pose_ = msg->pose;
		find_table_ = true;

		// if ((msg->pose.position.z) >= 1.1)
		// {
		// 	geometry_msgs::Twist cmd_msg;
		// 	cmd_msg.linear.x = 0.1;
		// 	base_cmd_pub.publish(cmd_msg);
		// }
		if ((msg->pose.position.z) <= 1.1 and !get_goal)
		{
			get_goal = true;
		}

	  // marker_transform.header.stamp = ros::Time::now();
	  // marker_transform.header.frame_id = msg->header.frame_id;
	  // marker_transform.child_frame_id = "marker_table";

	  // marker_transform.transform.translation.x = msg->pose.position.x;
	  // marker_transform.transform.translation.y = msg->pose.position.y;
	  // marker_transform.transform.translation.z = msg->pose.position.z;
	  // marker_transform.transform.rotation.x = msg->pose.orientation.x;
	  // marker_transform.transform.rotation.y = msg->pose.orientation.y;
	  // marker_transform.transform.rotation.z = msg->pose.orientation.z;
	  // marker_transform.transform.rotation.w = msg->pose.orientation.w;

	  // object_transform.header.stamp = ros::Time::now();
	  // object_transform.header.frame_id = "marker_table";
	  // object_transform.child_frame_id = "table";

	  // object_transform.transform.translation.x = 0.02;
	  // object_transform.transform.translation.y = -0.036;
	  // object_transform.transform.translation.z = 0.00;

	  // tf2::Quaternion q;
	  // q.setRPY(-M_PI/2.0, M_PI, -M_PI/2.0);
	  // object_transform.transform.rotation.x = q.x();
	  // object_transform.transform.rotation.y = q.y();
	  // object_transform.transform.rotation.z = q.z();
	  // object_transform.transform.rotation.w = q.w();
	  // tf2_ros::TransformBroadcaster br;
   //  br.sendTransform(marker_transform);
   //  br.sendTransform(object_transform);
    // ROS_INFO("table pose publishing");
  }

  void ExcuteThread(GoalHandle gh) {
		PickAndPlace(gh);
  }

	void ExecuteCb(GoalHandle gh)
	{
		ROS_INFO("````````````````````````````````````````");
		if (task_active_)
		{
			gh.setRejected();
			return;
		}
		gh.setAccepted();
		task_active_ = true;
		ROS_INFO("%c", gh.getGoal()->items);
	  ROS_INFO("call pick task server");
		// UpdateCabinet("table");
		// GetParam();
		// object_id_ = goal->items;
		// UpdateTable();
		// UpdateBox(88);
		std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
		thread.detach();
		task_active_ = false;
	}

  void preemptCB(GoalHandle gh)
  {
    
    // stop kinova API driver
    kinova_msgs::Stop stop_msg;
    kinova_msgs::Start start_msg;
    stop_client.call(stop_msg);
    start_client.call(start_msg);
    
		// cancel move_base goal
    get_goal = true;		
    move_base_client_.cancelAllGoals();

    // set the action state to preempted
    gh.setCanceled();
    task_active_ = false;
  }

  void GetParam() {

	  // get arm standby pose
	  arm_standby_pose_.resize(6);
	  nh_.getParam("arm_pose/standby", arm_standby_pose_);

	  // ROS_INFO("%f", arm_box_pose_[0][0]);

	  // get navigation position 
	  GetGoalPose("standby");
	  GetGoalPose("pick");
	  GetGoalPose("place");
  }

	void GetGoalPose(std::string goal_name) {

	  std::vector<float> pose_vector(3), orientation_vector(4);
	  nh_.getParam("target_goal/"+goal_name+"/pose", pose_vector);
	  nh_.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

	  geometry_msgs::Pose pose;
	  pose.position.x = pose_vector[0];
	  pose.position.y = pose_vector[1];
	  pose.position.z = pose_vector[2];
	  pose.orientation.x = orientation_vector[0];
	  pose.orientation.y = orientation_vector[1];
	  pose.orientation.z = orientation_vector[2];
	  pose.orientation.w = orientation_vector[3];

	  if (goal_name == "standby") {
	  	standby_position_ = pose;
	  } else if (goal_name == "pick") {
	  	pick_position_ = pose;
	  } else if (goal_name == "place") {
	  	place_position_ = pose;
	  }
	}

  void PickAndPlace(GoalHandle gh){
  	ROS_INFO("pick and place");

  	// get Move Group client
	  actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action = arm_group.getMoveGroupClient(); 
	  actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &gripper_group_action = gripper_group.getMoveGroupClient();
	  
	  geometry_msgs::PoseStamped target;
		kinova_msgs::SetFingersPositionGoal finger_position;
		pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
	  std::vector<std::string> object_ids;

	  // ros::WallDuration(1.0).sleep();
	  // std::vector<double> arm_box_pose(box_count_);
	  std::vector<double> arm_box_pose(6);
	  double box_position, box_size; 
	  int id;
  	std::string box_name = "box_information/" + gh.getGoal()->items;

  	nh_.getParam(box_name + "/arm_pose", arm_box_pose);
  	nh_.getParam(box_name + "/height", box_position);
  	nh_.getParam(box_name + "/size", box_size);

  	nh_.getParam(box_name+ "/id", id);
  	step = 1;
	  while(true) {
	  	ros::spinOnce();

			if (step == 1) {
				/* code */
				feedback_.step_description = "Set arm to ready pose";
				feedback_.step_index = 1;
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("setp1: Set arm to ready pose");
			  SetArmJointPose(arm_standby_pose_);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 2;
				feedback_.step_description = "Set body height to zero";
			}
			else if(step == 2) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step2: Set body height to zero");
			  SetHeight(0.0); 
				finger_position.fingers.finger1 = 6500; 
				finger_position.fingers.finger2 = 6500; 
				finger_position.fingers.finger3 = 0; 
				fc_.sendGoal(finger_position);      
				feedback_.step_index = 3;
				feedback_.step_description = "Move to standby position";
			}
			else if(step == 3) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
			  ROS_INFO("Move to standby position");
			  if (!MoveToGoal(pick_position_, gh))
			  {
			  	return;
			  }
			  get_goal = false;
			  ros::Rate loop_rate(10);

			  while(!get_goal) {
			  	geometry_msgs::Twist cmd_msg;
			  	cmd_msg.linear.x = 0.1;
			  	base_cmd_pub.publish(cmd_msg);
			  	loop_rate.sleep();
			  }
				feedback_.step_index = 4;
				feedback_.step_description = "Set pan tilt";
			}
			else if(step == 4) {
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step3: Set pan tilt");
		    pan_tilt_msgs.pitch = 0.0;
		    pan_tilt_msgs.yaw = 0.0;
		    pan_tilt_msgs.speed = 30;	
		    head_pub.publish(pan_tilt_msgs);
			  ros::WallDuration(3.5).sleep();
				feedback_.step_index = 5;
				feedback_.step_description = "Update cabinet collision";
			}
			else if(step == 5) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step6: Update cabinet collision");
		    if (find_table_) {
					UpdateCabinet("place name");
		    }
		    else {
		    	return;
		    }
				feedback_.step_index = 6; 
				feedback_.step_description = "Set height for pick";
			}
			else if(step == 6) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step5: Set height for pick");
			  ros::WallDuration(1.0).sleep();
			  SetHeight(box_position);       
				feedback_.step_index = 7;
				feedback_.step_description = "Set arm to standby pose";
			}
			else if(step == 7) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step7: Set arm to standby pose");
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("Open Gripper");
		    SetArmJointPose(arm_box_pose);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 8;
				feedback_.step_description = "Add object collision";
			}
			else if(step == 8) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step8: Add object collision");
				// delele collsion
				object_ids.push_back("cabinet_collision");
			  DeleltCollsion(object_ids);
			  object_ids.clear();
			  // add collsion on object link
			  UpdateBox(id);
		    // UpdateObject(box_size, id);
				feedback_.step_index = 9;
				feedback_.step_description = "Open gripper";
			}					
			else if(step == 9) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
				finger_position.fingers.finger1 = 0; 
				finger_position.fingers.finger2 = 0; 
				finger_position.fingers.finger3 = 0; 
				fc_.sendGoal(finger_position);
		    ROS_INFO("step9: Open gripper");
				feedback_.step_index = 10;
				feedback_.step_description = "Move arm to pre pick pose";
			}					
			else if(step == 10) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step10: Move arm to pre pick pose");
			  target.header.stamp = ros::Time::now();
			  target.header.frame_id = "object" + std::to_string(id);
			  target.pose.position.x = 0.0;
			  target.pose.position.y = 0.01;
			  target.pose.position.z = -0.22;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 11;
				feedback_.step_description = "Move arm to pick pose";
			}
			else if(step == 11) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step11: Move arm to pick pose");
			  ros::WallDuration(1.0).sleep();
			  target.header.stamp = ros::Time::now();
			  target.pose.position.z = -0.05;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 12;
				feedback_.step_description = "Close gripper";
			}
			else if(step == 12) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step10: Close gripper");
			  ros::WallDuration(1.0).sleep();
				finger_position.fingers.finger1 = 5000; 
				finger_position.fingers.finger2 = 5000; 
				finger_position.fingers.finger3 = 0; 
				fc_.sendGoal(finger_position);
				// arm_group.attachObject("object_collision_" + std::to_string(id));
				feedback_.step_index = 10;
				feedback_.step_description = "Move arm up to pick pose";
			}
			else if(step == 13) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step13: Move arm up to pick pose");
			  ros::WallDuration(1.0).sleep();
			  target.header.stamp = ros::Time::now();
				target.pose.position.y = 0.06;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 14;
				feedback_.step_description = "Move arm up to pick pose";
			}
			else if(step == 14) {
				gh.publishFeedback(feedback_);
		    ROS_INFO("step15: Move arm up to pick positio");
			  ros::WallDuration(1.0).sleep();
			  target.header.stamp = ros::Time::now();
				target.pose.position.z = -0.25;
				std::string name = gh.getGoal()->items;
				if (name == "box_1" or name == "box_4" or name == "box_7")
				{
					target.pose.position.z = -0.35;
				}
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				UpdateCabinet("place name");
				feedback_.step_index = 15;
				feedback_.step_description = "Move arm to standby pose";
			}					
			else if(step == 16) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step16: Move arm to standby pose");

			  // ros::Rate loop_rate(10);

		   //  for (int i = 0; i < 10; ++i)
		   //  {
			  // 	geometry_msgs::Twist cmd_msg_back;
			  // 	cmd_msg_back.linear.x = -0.1;
			  // 	base_cmd_pub.publish(cmd_msg_back);
			  // 	loop_rate.sleep();
		   //  }
			  SetArmJointPose(arm_standby_pose_);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }

			  //delete all collision
			 //  std::vector<std::string> object_ids;
				object_ids.push_back("cabinet_collision");
				object_ids.push_back("box_collision");
			  DeleltCollsion(object_ids);
			  object_ids.clear();
				feedback_.step_index = 17;
				feedback_.step_description = "Move arm up to pick pose";
			}					
			else if(step == 17) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step17: Set body to zero");
			  SetHeight(0.0);       
				feedback_.step_index = 17;
				feedback_.step_description = "Move to place position";
			}
			else if(step == 18) {
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step18: Move to place position");
			  if (!MoveToGoal(place_position_, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 13;
				feedback_.step_description = "Set pan tilt pitch to 30";
			}					
			else if(step == 19) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step19: Set pan tilt pitch to 30");
		    pan_tilt_msgs.pitch = 30.0;
		    pan_tilt_msgs.yaw = 0;
		    pan_tilt_msgs.speed = 30;
			  head_pub.publish(pan_tilt_msgs);
			  ros::WallDuration(2.0).sleep();
				feedback_.step_index = 19;
				feedback_.step_description = "Update table collision";
			}					
			else if(step == 20) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step20: Set body height to 0.15cm");
			  SetHeight(0.15); 
			  feedback_.step_index = 20;
				feedback_.step_description = "SetArmPlacePose";
			}					
			else if(step == 21) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step21: Updata table collision");
		    UpdateTable();
				feedback_.step_index = 21;
				feedback_.step_description = "Place object";
			}					
			else if(step == 22) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step22: Set arm to place pose");
			  target.header.stamp = ros::Time::now();
			  target.header.frame_id = "table2";
			  target.pose.position.x = -0.25;
			  target.pose.position.y = 0.15;
			  target.pose.position.z = -0.20;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 22;
				feedback_.step_description = "Place object down";
			}	
			else if(step == 23) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step23: Place object forward");
			  target.header.stamp = ros::Time::now();
			  target.pose.position.z = -0.05;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 23;
				feedback_.step_description = "Place object down";
			}	
			else if(step == 24) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step24: Place object down");
			  target.header.stamp = ros::Time::now();
			  target.pose.position.y = 0.09;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 24;
				feedback_.step_description = "open fingers";
			}	
			else if(step == 25) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step25: Open fingers");
				finger_position.fingers.finger1 = 0; 
				finger_position.fingers.finger2 = 0; 
				finger_position.fingers.finger3 = 0; 
				fc_.sendGoal(finger_position);
				// detach object from gripper
				// arm_group.detachObject("object_collision_" + std::to_string(id));
				feedback_.step_index = 25;
				feedback_.step_description = "Place object and away table";
			}	
			else if(step == 26) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step26: Place object and away table");
			  target.header.stamp = ros::Time::now();
			  target.pose.position.y = 0.15;
			  target.pose.position.z = -0.16;
				SetArmTargetPose(target);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 27;
				feedback_.step_description = "Back arm to standby pose";
			}	
			else if(step == 27) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step27: Back arm to standby pose");
			  SetArmJointPose(arm_standby_pose_);
			  if (!ArmMoveAndCheck(arm_group, arm_group_action, gh))
			  {
			  	return;
			  }
			  // delete table collision
			  std::vector<std::string> object_ids;
				// object_ids.push_back("object_collision_" + std::to_string(id));
				object_ids.push_back("table_collision");
			  DeleltCollsion(object_ids);
				feedback_.step_index = 28;
				feedback_.step_description = "Set pan tilt to zero";
			}	
			else if(step == 28) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step28: Set pan tilt to zero");
		    pan_tilt_msgs.pitch = 0.0;
		    pan_tilt_msgs.yaw = 0.0;
		    pan_tilt_msgs.speed = 30;
			  head_pub.publish(pan_tilt_msgs);
			  ros::WallDuration(1.5).sleep();
				feedback_.step_index = 29;
				feedback_.step_description = "Set height to zero and Close fingers for moving";
			}
			else if(step == 29) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step29: Set height to zero and Close fingers for moving");
		    SetHeight(0.0);
				finger_position.fingers.finger1 = 6500; 
				finger_position.fingers.finger2 = 6500; 
				finger_position.fingers.finger3 = 0; 
				fc_.sendGoal(finger_position);      
				feedback_.step_index = 30;
				feedback_.step_description = "Move to standby position";
			}		
			else if(step == 30) {
				gh.publishFeedback(feedback_);
			  ros::WallDuration(1.0).sleep();
		    ROS_INFO("step30: Move to standby position");
			  if (!MoveToGoal(standby_position_, gh))
			  {
			  	return;
			  }
				feedback_.step_index = 31;
				feedback_.step_description = "All finished";
			}	

			// if cancel
			if (gh.getGoalStatus().status == 2) {
				step = 1;
				return;
			}
			// if finished
			else if(step == 30) {
		    ROS_INFO("%i", step);
				// result_.finished_step = 7;
			  // gh.setSucceeded(result_, "all");
			  gh.setSucceeded();
		  	ROS_INFO("set succeeded");
				return;
			}
			step += 1;
	  }
  }

  //Set joint angle target,topic of "/joint_states"
	void SetArmJointPose(std::vector<double>& joint_value) {
	  arm_group.setJointValueTarget(joint_value);
	}

  //Set pose target,send pose msg to Moveit
	void SetArmTargetPose(geometry_msgs::PoseStamped target) {
	  arm_group.setPoseTarget(target);

	}

	//Set body height
	bool SetHeight(double height) {
	  if(height < 0.0) {
	    height = 0;
	  } else if (height > 0.4) {
	    height = 0.4;
	  }

	  ROS_INFO("%lf, %lf", height, body_height);

	  std_msgs::Float64 msg;
	  msg.data = height;
	  body_pub.publish(msg);

	  while(abs(body_height-height) > 0.01) {
	    ros::WallDuration(0.2).sleep();
	  }
	}

	// Using Moveit control grippers 
	void SetGripper(double box_size) {
	  std::vector<double> gripper_pose(2);

	  gripper_pose[0] = box_size*0.2;
	  gripper_pose[1] = box_size*0.2;
	  // gripper_pose[2] = value;

	  gripper_group.setJointValueTarget(gripper_pose);
	  // move_group.move();
	}
 
	// Using Moveit move arm and check execution
	bool ArmMoveAndCheck(moveit::planning_interface::MoveGroupInterface& move_group, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action, GoalHandle gh) {
	  arm_group.move();
	  if (arm_group_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	      ROS_INFO("SUCCEEDED");
  		  return true;
  	}
	  else {
				gh.publishFeedback(feedback_);
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
		    task_active_ = false;
        return false;
	      // ROS_INFO("Cancel Goal!");
	      // arm_group_action.cancelAllGoals();
	  }
	}

	// void SetArmNamePose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double>& joint_value) {
	//   move_group.setJointValueTarget(joint_value);
	//   move_group.move();
	// }

	// Using move_base control caster for navigation
	bool MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh) {
	  move_base_msgs::MoveBaseGoal mb_goal;
	  mb_goal.target_pose.header.stamp = ros::Time::now();
	  mb_goal.target_pose.header.frame_id = "map";
	  mb_goal.target_pose.pose = pose;

	  move_goal_flag = false;
	  move_base_client_.sendGoal(mb_goal,
	            boost::bind(&MovebaseDoneCallback, _1, _2),
	            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
	            boost::bind(&MovebaseFeedbackCallback, _1));

	  while(move_goal_flag==false) {
	    ros::WallDuration(1.0).sleep();
	    ROS_INFO_STREAM("moving...");
	  }

    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			  ROS_INFO_STREAM("get goal");
        // ROS_INFO("SUCCEEDED");
  		  return true;
  	}
    else {
        ROS_INFO("Cancel Goal!");
        move_base_client_.cancelAllGoals();
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
		    task_active_ = false;
    		return false;
    }
	}

	// Add cabinet collision
	void UpdateCabinet(std::string place_name) {

	  std::string collision_path = "package://caster_moma_app/collision_meshes/cabinet-urdf.stl";
	  shapes::Mesh *mesh=shapes::createMeshFromResource(collision_path);
	  if(mesh==NULL) {
	    ROS_INFO("search stl failed");
	  }
	  shapes::ShapeMsg shape_msg;
	  shapes::constructMsgFromShape(mesh, shape_msg);
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(1);

	  moveit_msgs::CollisionObject obj;

	  collision_objects[0].header.frame_id = "table";
	  collision_objects[0].id="cabinet_collision";

	  //定义物体方位
	  geometry_msgs::Pose pose;
	  // pose.position.z = -1.14;
	  pose.position.z = -1.13;
	  pose.position.x = 0.31;

		// tf2::Quaternion q;
  //   q.setRPY(M_PI/2.0, 0.0, M_PI/2.0);
  //   pose.orientation.x = q.x();
  //   pose.orientation.y = q.y();
  //   pose.orientation.z = q.z();
  //   pose.orientation.w = q.w();

	  // pose.position.x = 0.0;
	  // pose.position.y = table_pose_.position.y;
	  // pose.position.z = table_pose_.position.z;
	  // pose.orientation.x = table_pose_.orientation.x;
	  // pose.orientation.y = table_pose_.orientation.y;
	  // pose.orientation.z = table_pose_.orientation.z;
	  // pose.orientation.w = table_pose_.orientation.w;

	  //将形状添加到obj
	  collision_objects[0].meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
	  collision_objects[0].mesh_poses.push_back(pose);

	  //定义操作为添加
	  collision_objects[0].operation = collision_objects[0].ADD;
	  //定义一个PlanningScene消息
	  // moveit_msgs::PlanningScene planning_scene;
	  // planning_scene.world.collision_objects.push_back(collision_objects[0]);
	  // planning_scene.is_diff = true;
	  // //发布该消息
	  // planning_scene_diff_publisher.publish(planning_scene);
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	  ROS_INFO("add collision finished");

	}

	// Add box collision on object link
	void UpdateBox(int id) {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(2);

	  collision_objects[0].header.frame_id = "object" + std::to_string(id);
	  collision_objects[0].id = "box_collision";

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(3);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[1] = 0.01;
	  collision_objects[0].primitives[0].dimensions[0] = 1.0;
	  collision_objects[0].primitives[0].dimensions[2] = 0.32;
	  collision_objects[0].primitives[2].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[2].dimensions.resize(3);
	  collision_objects[0].primitives[2].dimensions[1] = 0.01;
	  collision_objects[0].primitives[2].dimensions[0] = 1.0;
	  collision_objects[0].primitives[2].dimensions[2] = 0.32;
	  collision_objects[0].primitives[1].type = collision_objects[0].primitives[1].BOX;
	  collision_objects[0].primitives[1].dimensions.resize(3);
	  collision_objects[0].primitives[1].dimensions[1] = 0.82;
	  collision_objects[0].primitives[1].dimensions[0] = 0.7;
	  collision_objects[0].primitives[1].dimensions[2] = 0.01;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(3);
	  collision_objects[0].primitive_poses[0].position.x = -0.35;
	  collision_objects[0].primitive_poses[0].position.y = -0.07;
	  collision_objects[0].primitive_poses[0].position.z = 0.01;

	  collision_objects[0].primitive_poses[2].position.x = -0.35;
	  collision_objects[0].primitive_poses[2].position.y = 0.33;
	  collision_objects[0].primitive_poses[2].position.z = 0.01;

	  collision_objects[0].primitive_poses[1].position.x = -0.53;
	  collision_objects[0].primitive_poses[1].position.y = -0.07;
	  collision_objects[0].primitive_poses[1].position.z = -0.16;

	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	// Add table collision
	void UpdateTable() {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(1);

	  collision_objects[0].header.frame_id = "table2";
	  collision_objects[0].id = "table_collision";

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(1);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[0] = 1.1;
	  collision_objects[0].primitives[0].dimensions[1] = 2.0;
	  collision_objects[0].primitives[0].dimensions[2] = 0.1;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(1);
	  tf2::Quaternion q;
    q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);

	  collision_objects[0].primitive_poses[0].position.x = -0.06;
	  collision_objects[0].primitive_poses[0].position.y = -0.05;
	  collision_objects[0].primitive_poses[0].position.z = 0.4;
	  collision_objects[0].primitive_poses[0].orientation.x = q.x();
	  collision_objects[0].primitive_poses[0].orientation.y = q.y();
	  collision_objects[0].primitive_poses[0].orientation.z = q.z();
	  collision_objects[0].primitive_poses[0].orientation.w = q.w();
	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	//Add object collision
	void UpdateObject(double box_size, int id) {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(1);

	  collision_objects[0].header.frame_id = "object" + std::to_string(id);
	  collision_objects[0].id = "object_collision_" + std::to_string(id);

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(1);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[0] = 0.05;
	  collision_objects[0].primitives[0].dimensions[1] = 0.06;
	  collision_objects[0].primitives[0].dimensions[2] = 0.05;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(1);

	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	// Delelt collision
	void DeleltCollsion(std::vector<std::string> object_ids) {
		planning_scene_interface.removeCollisionObjects(object_ids);
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_task_server");

  PickTaskAction averaging("pick_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  // return 0;
}