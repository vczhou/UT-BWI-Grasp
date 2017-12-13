#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>

//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/ArmJointAnglesAction.h"

// action definition
#include "bwi_grasp/GpdGraspAction.h"

//#include "agile_grasp/Grasps.h"
#include "bwi_grasp/GraspWithScoreList.h"

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

#include <geometry_msgs/TwistStamped.h>

#include <segbot_arm_manipulation/arm_utils.h>

using namespace std;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define PI 3.14159265

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300

#define NUM_JOINTS_ARMONLY 6
#define NUM_JOINTS 8 //6+2 for the arm

//some defines related to filtering candidate grasps
#define MAX_DISTANCE_TO_PLANE 0.075

//used when deciding whether a pair of an approach pose and a grasp pose are good;
//if the angular difference in joint space is too big, this means that the robot
//cannot directly go from approach to grasp pose (so we filter those pairs out)
#define ANGULAR_DIFF_THRESHOLD 3.0

//const float home_position [] = { -1.84799570991366, -0.9422852495301872, -0.23388692957209883, -1.690986384686938, 1.37682658669572, 3.2439323416434624};
const float home_position [] = {-1.9461704803383473, -0.39558648095261406, -0.6342860089305954, -1.7290658598495474, 1.4053863262257316, 3.039252699220428};
const float home_position_approach [] = {-1.9480954131742567, -0.9028227948134995, -0.6467984718381701, -1.4125267937404524, 0.8651278801122975, 3.73659131064558};

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

class GpdGraspActionServer {
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strage
    // error may occur.
    actionlib::SimpleActionServer<bwi_grasp::GpdGraspAction> as_;
    std::string action_name_;
    // TODO: figure out what result_ should be
    bwi_grasp::GpdGraspResult result_;
    
    sensor_msgs::JointState current_state;
    kinova_msgs::FingerPosition current_finger;
    geometry_msgs::PoseStamped current_pose;
    bool heardPose;
    bool heardJoinstState;
    
    bool heardGrasps;
    //agile_grasp::Grasps current_grasps; //change to new message type
    bwi_grasp::GraspWithScoreList current_grasps;
    
    //where we store results from calling the perception service
    // TODO: Figure out if actually used
    std::vector<PointCloudT::Ptr > detected_objects;
    
    geometry_msgs::PoseStamped current_moveit_pose;
    
    //store out-of-view position here
    sensor_msgs::JointState joint_state_outofview;
    geometry_msgs::PoseStamped pose_outofview;
    
    sensor_msgs::PointCloud2 cloud_ros;
    
    //publishers
    ros::Publisher pub_velocity;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_grasp_pub;
    ros::Publisher pose_array_pub;
    ros::Publisher pose_pub;
    ros::Publisher pose_fk_pub;
    
    // Used to compute transforms
    tf::TransformListener listener;
    
    bool heard_string;
    ros::Subscriber sub_string_;
    
    // Subscribers -- in action server need to be class members
    ros::Subscriber sub_angles;
    //    ros::Subscriber sub_torques;
    ros::Subscriber sub_tool;
    ros::Subscriber sub_finger;
    ros::Subscriber sub_grasps;
    
    struct GraspCartesianCommand {
        sensor_msgs::JointState approach_q;
        geometry_msgs::PoseStamped approach_pose;
        
        sensor_msgs::JointState grasp_q;
        geometry_msgs::PoseStamped grasp_pose;
        
        // stores the score assigned by gpd to that grasp
        std_msgs::Float32 score;	
    };
    
public:
    GpdGraspActionServer(std::string name) :
    as_(nh_, name, boost::bind(&GpdGraspActionServer::executeCB, this, _1), false),
    action_name_(name) {
        
        heardPose = false;
        heardJoinstState = false;
        heardGrasps = false;
        
        //create subscriber to joint angles
        sub_angles = nh_.subscribe ("/m1n6s200_driver/out/joint_state", 1,
				    &GpdGraspActionServer::joint_state_cb, 
				    this);
        
        //create subscriber to tool position topic
        sub_tool = nh_.subscribe("/m1n6s200_driver/out/tool_pose", 1, &GpdGraspActionServer::toolpos_cb, this);
        
        //subscriber for fingers
        sub_finger = nh_.subscribe("/m1n6s200_driver/out/finger_position", 1, &GpdGraspActionServer::fingers_cb, this);
        
        //subscriber for grasps
        //change to our topic
        //sub_grasps = n.subscribe("/find_grasps/grasps_handles",1, grasps_cb);
        sub_grasps = nh_.subscribe("/bwi_grasp/grasps", 1, &GpdGraspActionServer::grasps_cb, this);
        
        //publish velocities
        pub_velocity = nh_.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
        
        //publish pose array
        pose_array_pub = nh_.advertise<geometry_msgs::PoseArray>("/agile_grasp_demo/pose_array", 10);
        
        //publish pose
        pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_out", 10);
        pose_fk_pub = nh_.advertise<geometry_msgs::PoseStamped>("/agile_grasp_demo/pose_fk_out", 10);
        
        //debugging publisher
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud_debug", 10);
        cloud_grasp_pub = nh_.advertise<sensor_msgs::PointCloud2>("agile_grasp_demo/cloud", 10);
        
        ROS_INFO("Starting gpd grasp action server...");
        as_.start();
    }
    
    ~GpdGraspActionServer(void) {}
    
    // Blocking call for user input
    void pressEnter(){
        std::cout << "Press the ENTER key to continue";
        while (std::cin.get() != '\n')
            std::cout << "Please press ENTER\n";
    }
    
    // Joint state cb
    void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
        
        if (input->position.size() == NUM_JOINTS){
            current_state = *input;
            heardJoinstState = true;
        }
        //ROS_INFO_STREAM(current_state);
    }
    
    // Tool position cb
    void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
        current_pose = msg;
        heardPose = true;
        //  ROS_INFO_STREAM(current_pose);
    }
    
    // Finger state cb
    void fingers_cb (const kinova_msgs::FingerPosition msg) {
        current_finger = msg;
    }
    
    void grasps_cb(const bwi_grasp::GraspWithScoreList &msg){
        current_grasps = msg;
        heardGrasps = true;
    }
    
    void listenForArmData(float rate){
        heardPose = false;
        heardJoinstState = false;
        ros::Rate r(rate);
        
        while (ros::ok()){
            ros::spinOnce();
            
            if (heardPose && heardJoinstState)
                return;
            
            r.sleep();
        }
    }
    
    void listenForGrasps(float rate){
        ros::Rate r(rate);
        
        while (ros::ok()){
            ros::spinOnce();
            if (heardGrasps) {
                ROS_INFO_STREAM("Listen for grasps terminating");
                return;
            }
            
            r.sleep();
        }
    }
    
    double angular_difference(geometry_msgs::Quaternion c,
                              geometry_msgs::Quaternion d) {
        Eigen::Vector4f dv;
        dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
        Eigen::Matrix<float, 3,4> inv;
        inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
        inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = c.w;	inv(1,3) = -c.x;
        inv(2,0) = -c.z; inv(2,1) = -c.y;inv(2,2) = c.x;  inv(2,3) = c.w;
        
        Eigen::Vector3f m = inv * dv * -2.0;
        return m.norm();
    }
    
    int selectObjectToGrasp(std::vector<PointCloudT::Ptr > candidates) {
        //currently, we just pick the one with the most points
        int max_num_points = -1;
        int index = -1;
        
        for (unsigned int i = 0; i < candidates.size(); i ++){
            if ((int)candidates.at(i)->points.size() > max_num_points){
                max_num_points = (int)candidates.at(i)->points.size();
                index = (int)i;
                
            }
        }
        
        return index;
    }
    
    Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d& Q) {
        std::vector<int> axis_order_;
        axis_order_.push_back(2);
        axis_order_.push_back(0);
        axis_order_.push_back(1);
        
        Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
        R.col(axis_order_[0]) = Q.col(0); // grasp approach vector
        R.col(axis_order_[1]) = Q.col(1); // hand axis
        R.col(axis_order_[2]) = Q.col(2); // hand binormal
        return R;
    }
    
    geometry_msgs::PoseStamped graspToPose(bwi_grasp::GraspWithScore grasp,
                                           double hand_offset,
                                           std::string frame_id) {
        
        Eigen::Vector3d center_; // grasp position
        Eigen::Vector3d surface_center_; //  grasp position projected back onto the surface of the object
        Eigen::Vector3d axis_; //  hand axis
        Eigen::Vector3d approach_; //  grasp approach vector
        Eigen::Vector3d binormal_; //  vector orthogonal to the hand axis and the grasp approach direction
        
        tf::vectorMsgToEigen(grasp.axis, binormal_);
        tf::vectorMsgToEigen(grasp.approach, approach_);
        tf::vectorMsgToEigen(grasp.center, center_);
        tf::vectorMsgToEigen(grasp.surface_center, surface_center_);
        
        approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
        axis_ = binormal_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
        
        //step 1: calculate hand orientation
        
        // rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
        Eigen::Transform<double, 3, Eigen::Affine> T_R(Eigen::AngleAxis<double>(M_PI/2, approach_));
        
        //to do: compute the second possible grasp by rotating -M_PI/2 instead
        
        // calculate first hand orientation
        Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
        R.col(0) = -1.0 * approach_;
        R.col(1) = T_R * axis_;
        R.col(2) << R.col(0).cross(R.col(1));
        
        Eigen::Matrix3d R1 = reorderHandAxes(R);
        tf::Matrix3x3 TF1;
        tf::matrixEigenToTF(R1, TF1);
        tf::Quaternion quat1;
        TF1.getRotation(quat1);
        quat1.normalize();
        
        // rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
        /*Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(M_PI, approach_));
         
         // calculate second hand orientation
         Eigen::Matrix3d Q = Eigen::MatrixXd::Zero(3, 3);
         Q.col(0) = T * approach_;
         Q.col(1) = T * axis_;
         Q.col(2) << Q.col(0).cross(Q.col(1));
         
         // reorder rotation matrix columns according to axes ordering of the robot hand
         //Eigen::Matrix3d R1 = reorderHandAxes(R);
         Eigen::Matrix3d R2 = reorderHandAxes(Q);
         
         // convert Eigen rotation matrices to TF quaternions and normalize them
         tf::Matrix3x3 TF2;
         tf::matrixEigenToTF(R2, TF2);
         tf::Quaternion quat2;
         TF2.getRotation(quat2);
         quat2.normalize();
         
         std::vector<tf::Quaternion> quats;
         quats.push_back(quat1);
         quats.push_back(quat2);*/
        
        //use the first quaterneon for now
        tf::Quaternion quat = quat1;
        
        //angles to try
        double theta = 0.0;
        
        // calculate grasp position
        Eigen::Vector3d position;
        Eigen::Vector3d approach = -1.0 * approach_;
        if (theta != 0)
        {
            // project grasp bottom position onto the line defined by grasp surface position and approach vector
            Eigen::Vector3d s, b, a;
            position = (center_ - surface_center_).dot(approach) * approach;
            position += surface_center_;
        }
        else
            position = center_;
        
        // translate grasp position by <hand_offset_> along the grasp approach vector
        position = position + hand_offset * approach;
        
        geometry_msgs::PoseStamped pose_st;
        pose_st.header.stamp = ros::Time(0);
        pose_st.header.frame_id = frame_id;
        tf::pointEigenToMsg(position, pose_st.pose.position);
        tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
        
        return pose_st;
    }
    
    bool acceptGrasp(GraspCartesianCommand gcc,
                     /*PointCloudT::Ptr object,*/
                     Eigen::Vector4f plane_c){
        //filter 1: if too close to the plane
        pcl::PointXYZ p_a;
        p_a.x=gcc.approach_pose.pose.position.x;
        p_a.y=gcc.approach_pose.pose.position.y;
        p_a.z=gcc.approach_pose.pose.position.z;
        
        pcl::PointXYZ p_g;
        p_g.x=gcc.grasp_pose.pose.position.x;
        p_g.y=gcc.grasp_pose.pose.position.y;
        p_g.z=gcc.grasp_pose.pose.position.z;
        
        if (pcl::pointToPlaneDistance(p_a, plane_c) < MAX_DISTANCE_TO_PLANE
            || pcl::pointToPlaneDistance(p_g, plane_c) < MAX_DISTANCE_TO_PLANE){
            ROS_INFO_STREAM("Plane distance too short");
            return false;
        }
        
        return true;
    }
    
    moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n,
                                                   geometry_msgs::PoseStamped p) {
        ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
        
        moveit_msgs::GetPositionIK::Request ikine_request;
        moveit_msgs::GetPositionIK::Response ikine_response;
        ikine_request.ik_request.group_name = "arm";
        ikine_request.ik_request.pose_stamped = p;
        
        /* Call the service */
        if(ikine_client.call(ikine_request, ikine_response)){
            ROS_INFO("IK service call success:");
            ROS_INFO_STREAM(ikine_response);
        } else {
            ROS_INFO("IK service call FAILED. Exiting");
        }
        
        return ikine_response;
    }
    
    void spinSleep(double duration) {
        int rateHertz = 40;
        
        ros::Rate r(rateHertz);
        for(int i = 0; i < (int)duration * rateHertz; i++) {
            ros::spinOnce();
            r.sleep();
        }
    }
    
    void updateFK(ros::NodeHandle n) {
        ros::ServiceClient fkine_client = n.serviceClient<moveit_msgs::GetPositionFK> ("/compute_fk");
        
        moveit_msgs::GetPositionFK::Request fkine_request;
        moveit_msgs::GetPositionFK::Response fkine_response;
        
        
        //wait to get lates joint state values
        listenForArmData(30.0);
        sensor_msgs::JointState q_true = current_state;
        
        //Load request with the desired link
        fkine_request.fk_link_names.push_back("m1n6s200_end_effector");
        
        //and the current frame
        fkine_request.header.frame_id = "m1n6s200_link_base";
        
        //finally we let moveit know what joint positions we want to compute
        //in this case, the current state
        fkine_request.robot_state.joint_state = q_true;
        
        ROS_INFO("Making FK call");
        if(fkine_client.call(fkine_request, fkine_response)){
            pose_fk_pub.publish(fkine_response.pose_stamped.at(0));
            ros::spinOnce();
            current_moveit_pose = fkine_response.pose_stamped.at(0);
            ROS_INFO("Call successful. Response:");
            ROS_INFO_STREAM(fkine_response);
        } else {
            // TODO -- set as error?
            ROS_WARN("Call failed. Terminating.");
            as_.setAborted(result_);
            return;
            //ros::shutdown();
        }
        
    }
    
    void cartesianVelocityMove(double dx, double dy, double dz, double duration) {
        int rateHertz = 40;
        kinova_msgs::PoseVelocity velocityMsg;
        
        ros::Rate r(rateHertz);
        for(int i = 0; i < (int)duration * rateHertz; i++) {
            
            velocityMsg.twist_linear_x = dx;
            velocityMsg.twist_linear_y = dy;
            velocityMsg.twist_linear_z = dz;
            
            velocityMsg.twist_angular_x = 0.0;
            velocityMsg.twist_angular_y = 0.0;
            velocityMsg.twist_angular_z = 0.0;
            
            
            pub_velocity.publish(velocityMsg);
            ROS_INFO("Published cartesian vel. command");
            r.sleep();
        }
    }
    
    //lifts ef specified distance
    void lift_velocity(double vel, double distance){
        double pubRate = 40.0;
        ros::Rate r(pubRate);
        ros::spinOnce();
        double distance_init = .2;
        kinova_msgs::PoseVelocity T;
        T.twist_linear_x= 0.0;
        T.twist_linear_y= 0.0;
        T.twist_angular_x= 0.0;
        T.twist_angular_y= 0.0;
        T.twist_angular_z= 0.0;
        
        for(int i = 0; i < std::abs(pubRate*distance/vel); i++){
            ros::spinOnce();
            if(distance > 0)
                T.twist_linear_z = vel;
            else
                T.twist_linear_z= -vel;
            pub_velocity.publish(T);
            r.sleep();
        }
        T.twist_linear_z= 0.0;
        pub_velocity.publish(T);
    }
    
    void move_with_cartesian_velocities(const kinova_msgs::PoseVelocity &velocities,
                                        const double seconds) {
        // Per the Kinova documentation, we have to publish at exactly 100Hz to get
        // predictable behavior
        ros::Rate r(100);
        
        ros::Time end = ros::Time::now() + ros::Duration(seconds);
        while (ros::ok()) {
            //collect messages
            ros::spinOnce();
            
            //publish velocity message
            pub_velocity.publish(velocities);
            r.sleep();
            if (ros::Time::now() > end) {
                break;
            }
        }
        kinova_msgs::PoseVelocity zeros;
        pub_velocity.publish(zeros);
        
    }
    
    void lift(ros::NodeHandle n, double x) {
        listenForArmData(30.0);
        
        geometry_msgs::PoseStamped p_target = current_pose;
        
        p_target.pose.position.z += x;
        segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
    }

    void executeCB(const bwi_grasp::GpdGraspGoalConstPtr &goal) {
        std::string joint_state_topic = "/m1n6s200_driver/out/joint_state";
        while (true) {
            boost::shared_ptr<const sensor_msgs::JointState> result = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic, nh_, ros::Duration(5.0));
            if (result) {
                break;
            }
            ROS_WARN("Could not read joint states, retrying...");
        }
        
        // user input
        char in;
        
        ROS_INFO("Demo starting...move the arm to a position where it is not occluding the table.");
        //TODO This is where we have to change the file
        pressEnter();
        
        ROS_INFO("Got user input");
        
        //store out of table joint position
        listenForArmData(30.0);
        joint_state_outofview = current_state;
        pose_outofview = current_pose;
        
        ROS_INFO("Stored current joint position");
        
        segbot_arm_manipulation::openHand();
        
        // gpd will do its own picking up which object to pick up using RGBD image data
        
        ROS_INFO("Getting tabletop scene...");
        
        segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(nh_);
        
        ROS_INFO("calculated tabletop scene");
        //step 2: extract the data from the response
        /*detected_objects.clear();
         for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
         PointCloudT::Ptr object_i (new PointCloudT);
         pcl::PCLPointCloud2 pc_i;
         pcl_conversions::toPCL(table_scene.cloud_clusters.at(i),pc_i);
         pcl::fromPCLPointCloud2(pc_i,*object_i);
         detected_objects.push_back(object_i);
         }
         
         if (detected_objects.size() == 0){
         ROS_WARN("[agile_grasp_demo.cpp] No objects detected...aborting.");
         return 1;
         }*/
        
        Eigen::Vector4f plane_coef_vector;
        for (int i = 0; i < 4; i ++)
            plane_coef_vector(i)=table_scene.cloud_plane_coef[i];
        
        //step 3: select which object to grasp
        /*int selected_object = selectObjectToGrasp(detected_objects);
         
         //publish object to find grasp topic
         pcl::PCLPointCloud2 pc_target;
         pcl::toPCLPointCloud2(*detected_objects.at(selected_object),pc_target);
         pcl_conversions::fromPCL(pc_target,cloud_ros);
         
         //publish to agile_grasp
         ROS_INFO("Publishing point cloud...");
         cloud_grasp_pub.publish(cloud_ros);*/
        
        // listen to our node instead of agile grasp
        //wait for response at 30 Hz
        listenForGrasps(30.0);
        
        geometry_msgs::PoseArray poses_msg;
        poses_msg.header.seq = 1;
        poses_msg.header.stamp = cloud_ros.header.stamp;
        poses_msg.header.frame_id = "m1n6s200_link_base";
        
        ROS_INFO("[agile_grasp_demo.cpp] Heard %i grasps",(int)current_grasps.grasps.size());
        
        //next, compute approach and grasp poses for each detected grasp
        double hand_offset_grasp = -0.02;
        double hand_offset_approach = -0.13;
        
        //wait for transform from visual space to arm space
        ROS_INFO("Waiting for transform...");
        listener.waitForTransform( "/base_footprint","m1n6s200_link_base", ros::Time(0), ros::Duration(5.0));
        
        std::vector<GraspCartesianCommand> grasp_commands;
        std::vector<float> scores;
        std::vector<geometry_msgs::PoseStamped> poses;
        for (unsigned int i = 0; i < current_grasps.grasps.size(); i++) {
            geometry_msgs::PoseStamped p_grasp_i = graspToPose(current_grasps.grasps.at(i),hand_offset_grasp,"xtion_camera_rgb_optical_frame");
            geometry_msgs::PoseStamped p_approach_i = graspToPose(current_grasps.grasps.at(i),hand_offset_approach,"xtion_camera_rgb_optical_frame");
            
            ROS_INFO_STREAM("Checking grasp...");
            ROS_INFO_STREAM(current_grasps.grasps.at(i));
            GraspCartesianCommand gc_i;
            gc_i.approach_pose = p_approach_i;
            gc_i.grasp_pose = p_grasp_i;
            ROS_INFO_STREAM("Grasp turned into pose...");
            ROS_INFO_STREAM("Approach: \n" << gc_i.approach_pose << "\nGrasp: \n" << gc_i.approach_pose);
            
            if (acceptGrasp(gc_i,/*detected_objects.at(selected_object),*/plane_coef_vector)){
                
                ROS_INFO_STREAM("Accepted grasp");
                listener.transformPose("m1n6s200_link_base", gc_i.approach_pose, gc_i.approach_pose);
                listener.transformPose("m1n6s200_link_base", gc_i.grasp_pose, gc_i.grasp_pose);
                
                //filter two -- if IK fails
                moveit_msgs::GetPositionIK::Response  ik_response_approach = computeIK(nh_, gc_i.approach_pose);
                if (ik_response_approach.error_code.val == 1){
                    ROS_INFO_STREAM("Got past second check");
                    moveit_msgs::GetPositionIK::Response  ik_response_grasp = computeIK(nh_, gc_i.grasp_pose);
                    
                    if (ik_response_grasp.error_code.val == 1){
                        ROS_INFO_STREAM("Got past third check");
                        
                        //now check to see how close the two sets of joint angles are
                        std::vector<double> D = segbot_arm_manipulation::getJointAngleDifferences(ik_response_approach.solution.joint_state, ik_response_grasp.solution.joint_state );
                        
                        double sum_d = 0;
                        for (int p = 0; p < D.size(); p++){
                            sum_d += D[p];
                        }
                        
                        if (sum_d < ANGULAR_DIFF_THRESHOLD){
                            ROS_INFO("Angle diffs for grasp %i: %f, %f, %f, %f, %f, %f",(int)grasp_commands.size(),D[0],D[1],D[2],D[3],D[4],D[5]);
                            
                            ROS_INFO("Sum diff: %f",sum_d);
                            
                            //store the IK results
                            gc_i.approach_q = ik_response_approach.solution.joint_state;
                            gc_i.grasp_q = ik_response_grasp.solution.joint_state;
                            
                            //add score
                            scores.push_back(current_grasps.grasps.at(i).score.data);
                            
                            grasp_commands.push_back(gc_i);
                            poses.push_back(p_grasp_i);
                            poses_msg.poses.push_back(gc_i.approach_pose.pose);
                        }
                    }
                }
            }
            
        }
        
        //make sure we're working with the correct tool pose
        listenForArmData(30.0);
        ROS_INFO("[agile_grasp_demo.cpp] Heard arm pose.");
        
        //now, select the target grasp
        updateFK(nh_);
        
        //find the grasp with closest orientation to current pose
        /*double min_diff = 1000000.0;
         int min_diff_index = -1;
         
         for (unsigned int i = 0; i < grasp_commands.size(); i++){
         double d_i = angular_difference(grasp_commands.at(i).approach_pose.pose.orientation, current_pose.pose.orientation);
         
         ROS_INFO("Distance for pose %i:\t%f",(int)i,d_i);
         if (d_i < min_diff){
         min_diff_index = (int)i;
         min_diff = d_i;
         }
         }*/
        
        ROS_INFO_STREAM("Number of grasps after processing is " << grasp_commands.size());
        
        // pick the grasp with the highest score
        if (grasp_commands.size() != scores.size()) {
            ROS_WARN("Number of grasps doesn't equal number of scores");
        }
        if (grasp_commands.size() == 0) {
            ROS_WARN("No feasible grasps founds, aborting.");
            as_.setAborted(result_);
            return;
        }
        
        // always pick first element in array because this will have highest score
        int max_score_index = 0;
        double max_score = 0;
        for (int i = 0; i < scores.size(); i++) {
            if (scores.at(i) > max_score) {
                max_score_index = i;
                max_score = scores.at(i);
            }
        }
        ROS_INFO("Picking grasp %i with score %f...",max_score_index, max_score);
        
        pose_array_pub.publish(poses_msg);
        
        //publish individual pose
        pose_pub.publish(grasp_commands.at(max_score_index).approach_pose);
        ROS_INFO_STREAM(grasp_commands.at(max_score_index).approach_q);
        pressEnter();
        
        //set obstacle avoidance
        /*std::vector<sensor_msgs::PointCloud2> obstacle_clouds;
         obstacle_clouds.push_back(cloud_ros);
         segbot_arm_manipulation::setArmObstacles(nh_,obstacle_clouds);*/
        
        segbot_arm_manipulation::moveToPoseMoveIt(nh_, grasp_commands.at(max_score_index).approach_pose);
        
        //clear obstacles for final approach
        /*obstacle_clouds.clear();
         obstacle_clouds.push_back(table_scene.cloud_plane);
         segbot_arm_manipulation::setArmObstacles(nh_,obstacle_clouds);*/
        
        segbot_arm_manipulation::moveToPoseMoveIt(nh_, grasp_commands.at(max_score_index).approach_pose);
        
        //pressEnter();
        
        //open fingers
        //pressEnter();
        //moveFinger(100);
        
        //sleep(1.0);
        //pose_pub.publish(grasp_commands.at(min_diff_index).grasp_pose);
        /*std::cout << "Press '1' to move to approach pose or Ctrl-z to quit..." << std::endl;		
         std::cin >> in;*/
        
        segbot_arm_manipulation::moveToPoseMoveIt(nh_, grasp_commands.at(max_score_index).grasp_pose);
        spinSleep(3.0);
        
        //listenForArmData(30.0);
        //close fingers
        //pressEnter();
        
        //moveFinger(7200);
        segbot_arm_manipulation::closeHand();
        
        //lift for a while
        //pressEnter();
        ROS_INFO_STREAM("Attempting to lift...");
        
        kinova_msgs::PoseVelocity velocities;
        velocities.twist_linear_z = .2;
        move_with_cartesian_velocities(velocities, 3);
        //lift_velocity(0.2,5.0); 
        
        //we need to call angular velocity control before we can do cartesian control right after using the fingers
        segbot_arm_manipulation::openHand();
        
        //TODO actually give velocity commands instead of calling lift
        
        //moveToPoseMoveIt(nh_,pose_outofview);
        //moveToPoseMoveIt(nh_,pose_outofview);
        
        segbot_arm_manipulation::homeArm(nh_);
        
        segbot_arm_manipulation::moveToJointState(nh_,joint_state_outofview);
        segbot_arm_manipulation::moveToJointState(nh_,joint_state_outofview);
        
        //moveToJointState(home_position_approach);
        //moveToJointState(home_position);
        result_.success = true;
        as_.setSucceeded(result_);
    }
};

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpd_grasp_as");
    
    //register ctrl-c
    signal(SIGINT, sig_handler);
    
    GpdGraspActionServer as(ros::this_node::getName());
    ros::spin();
    
    return 0;
}

