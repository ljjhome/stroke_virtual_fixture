#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
void robot_home(ros::Publisher &pub);

int main(int argc, char** argv){
    
    ros::init(argc, argv, "my_cmd_publisher");

    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);

    tf::TransformListener listener;
    ros::Rate loop_rate(10);
    int count = 0;
    tf::StampedTransform transform; 
    tf::Vector3 vec;
    robot_home(pub);
    /* robot home function test*/
    trajectory_msgs::JointTrajectory traj2;
    trajectory_msgs::JointTrajectoryPoint trajpoint2;
    traj2.joint_names.push_back("shoulder_pan_joint");
    traj2.joint_names.push_back("shoulder_lift_joint");
    traj2.joint_names.push_back("elbow_joint");
    traj2.joint_names.push_back("wrist_1_joint");
    traj2.joint_names.push_back("wrist_2_joint");
    traj2.joint_names.push_back("wrist_3_joint");

    trajpoint2.positions.push_back(0);
    
    trajpoint2.positions.push_back(-1.0);
    trajpoint2.positions.push_back(0.8);
    trajpoint2.positions.push_back(-2.2);
    trajpoint2.positions.push_back(-1.8);
    trajpoint2.positions.push_back(0.0);

    ros::Duration duration2(4.0);
    trajpoint2.time_from_start = duration2; 
    traj2.points.push_back(trajpoint2);
    while(count<50){
        count++;
        pub.publish(traj2);
        loop_rate.sleep();
    }
    count = 0;
    while (count<20){
        count++;

        try{
            listener.lookupTransform("/base_link","/ee_link",ros::Time(0),transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        vec = transform.getOrigin();
        ROS_INFO("Origin:x %f y %f z %f",vec.getX(),vec.getY(),vec.getZ());
    }

    /* some parameters and settings for the ik solver*/ 
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    double eps = 1e-5;
    node.param("chain_start",chain_start,std::string("base_link"));
    node.param("chain_end", chain_end, std::string("ee_link"));

    if (chain_start ==""||chain_end==""){
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }
    node.param("timeout",timeout, 0.005);
    node.param("urdf_param",urdf_param,std::string("/robot_description"));

    TRAC_IK::TRAC_IK tracik_solver(chain_start,chain_end, urdf_param,timeout,eps);

    KDL::Chain chain;
    KDL::JntArray ll,ul;// lower and uper joint limits
    for (uint tt = 0;tt<ll.data.size();tt++){
        ll(tt) = -3;
        ul(tt) = 3;
    } 
    bool valid = tracik_solver.getKDLChain(chain);
    ROS_INFO("hahahha");
    if (!valid){
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }
    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid){
        ROS_ERROR("There was no valid KDL joint limit found");
        return -1;
    }

    assert(chain.getNrOfJoints()==ll.data.size());
    assert(chain.getNrOfJoints()==ul.data.size());

    ROS_INFO("using %d joints",chain.getNrOfJoints());

    KDL::JntArray nominal(chain.getNrOfJoints());
    for (uint j = 0;j<nominal.data.size();j++){
        nominal(j) = (ll(j)+ul(j))/2.0; 
    }

    KDL::JntArray result;
    KDL::Frame end_effector_pose;
    int rc; 
    transform.setOrigin(transform.getOrigin() + transform.getOrigin().normalized() * (-0.2));
    tf::transformTFToKDL(transform,end_effector_pose);
    rc = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    if (rc<0){
        ROS_INFO("fuck no found");
        return -1; 
    }
    // create the joint control msg we need to publish
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint trajpoint;
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    trajpoint.positions.push_back(result(0));
    trajpoint.positions.push_back(result(1));
    trajpoint.positions.push_back(result(2));
    trajpoint.positions.push_back(result(3));
    trajpoint.positions.push_back(result(4));
    trajpoint.positions.push_back(result(5));

    ros::Duration duration(5.0);
    trajpoint.time_from_start = duration; 
    traj.points.push_back(trajpoint);

    while(ros::ok()){
        pub.publish(traj);
        loop_rate.sleep();        
    }

}
void robot_home(ros::Publisher &pub){
    
    trajectory_msgs::JointTrajectory traj2;
    trajectory_msgs::JointTrajectoryPoint trajpoint2;
    traj2.joint_names.push_back("shoulder_pan_joint");
    traj2.joint_names.push_back("shoulder_lift_joint");
    traj2.joint_names.push_back("elbow_joint");
    traj2.joint_names.push_back("wrist_1_joint");
    traj2.joint_names.push_back("wrist_2_joint");
    traj2.joint_names.push_back("wrist_3_joint");

    trajpoint2.positions.push_back(0);
    
    trajpoint2.positions.push_back(-1.0);
    trajpoint2.positions.push_back(0.8);
    trajpoint2.positions.push_back(-2.2);
    trajpoint2.positions.push_back(-1.8);
    trajpoint2.positions.push_back(0.0);

    ros::Duration duration(2.0);
    trajpoint2.time_from_start = duration; 
    traj2.points.push_back(trajpoint2);
    pub.publish(traj2);
    ros::Duration sleep(4);
    ROS_INFO("after home"); 
}
