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
#include <geometry_msgs/Vector3.h>
#include "rosqt/setVF.h"
#define PI 3.141592654
double MaxForce = 3; //the maximum force the manipulator can afford
double vf_handle_length;
int vf_handle_rot_dir;
double door_plane_offset = 0.02; //unit : m
bool move_to_start_pos = false;
trajectory_msgs::JointTrajectory traj;
trajectory_msgs::JointTrajectoryPoint trajpoint;
tf::Vector3 move_vec;
tf::StampedTransform transform; 
tf::StampedTransform start_transform; 
tf::StampedTransform transform_kinect; 
tf::Vector3 handle_leftpoint, handle_rightpoint;
tf::Vector3 robot_startpoint;
void strokeCallback(const geometry_msgs::Vector3 &msg){
    double move_length, k;
    k = 1;
    tf::Vector3 move_dir(msg.x,msg.y,msg.z); 
    if (move_dir.length()>40){
        move_length = k* (move_dir.length());
        move_vec = move_length * move_dir.normalize();
    }else{
        move_vec.setValue(0.0,0.0,0.0); 
    }
}

tf::Vector3 computeMotion(tf::Vector3 leftvec,tf::Vector3 rightvec,int rot_d, tf::Vector3 move_vec,tf::StampedTransform tran, double tipforce);
/* In this function, we have to make sure all the point are 
 * with respect to the same frame*/
bool setVFparam(rosqt::setVF::Request &req, 
                rosqt::setVF::Response &res){
    //try{
    //    listener.lookupTransform("/base_link","/kinect_link",ros::Time(0),transform);
    //}
    //catch (tf::TransformException &ex){
    //    ROS_ERROR("%s",ex.what());
    //    ros::Duration(1.0).sleep();
    //}
    move_to_start_pos = req.initial_move;
    tf::Vector3 handle_leftpoint_tmp, handle_rightpoint_tmp, robot_startpoint_tmp;
    vf_handle_length = req.length;
    vf_handle_rot_dir = req.rot_dir;
    handle_leftpoint_tmp = tf::Vector3(req.leftp.x,req.leftp.y,1 * req.leftp.z);
    handle_rightpoint_tmp = tf::Vector3(req.rightp.x,req.rightp.y,1 * req.rightp.z);
    robot_startpoint_tmp = tf::Vector3(req.robot_startP.x,req.robot_startP.y,1* req.robot_startP.z);
    res.success = true;
    ROS_INFO("handle_length: %f, rot_dir: %d",vf_handle_length,vf_handle_rot_dir);
    tf::Vector3 tmpori;
    tf::Matrix3x3 rotMat(-1,0,0,0,0,-1,0,-1,0);
    tmpori = transform_kinect.getOrigin();
    tf::Transform trans_tmp(rotMat,tmpori);
    robot_startpoint = trans_tmp*robot_startpoint_tmp;
    robot_startpoint.setX(robot_startpoint.x());
    robot_startpoint.setY(robot_startpoint.y() +0.12);
    robot_startpoint.setZ(robot_startpoint.z() + 0.20);
    handle_leftpoint = trans_tmp * handle_leftpoint_tmp;
    handle_leftpoint.setX(handle_leftpoint.x());
    handle_leftpoint.setY(handle_leftpoint.y() +0.12);
    handle_leftpoint.setZ(handle_leftpoint.z() + 0.20);
    handle_rightpoint = trans_tmp * handle_rightpoint_tmp;
    handle_rightpoint.setX(handle_rightpoint.x());
    handle_rightpoint.setY(handle_rightpoint.y() +0.12);
    handle_rightpoint.setZ(handle_rightpoint.z() + 0.20);

    ROS_INFO("origin in kinect: %f, %f, %f", handle_leftpoint_tmp.x(),handle_leftpoint_tmp.y(),handle_leftpoint_tmp.z());
    ROS_INFO("origin in base: %f, %f, %f", handle_leftpoint.x(),handle_leftpoint.y(),handle_leftpoint.z());
    return true;
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle node;

    ros::Publisher cmdpub = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
    ros::Subscriber stksub = node.subscribe("/drive_stroke",1, strokeCallback);
    ros::ServiceServer  service = node.advertiseService("setVF",setVFparam);

    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");
    trajpoint.positions.push_back(0.0);
    trajpoint.positions.push_back(0.0);
    trajpoint.positions.push_back(0.0);
    trajpoint.positions.push_back(0.0);
    trajpoint.positions.push_back(0.0);
    trajpoint.positions.push_back(0.0);

    tf::TransformListener listener;
    ros::Duration duration(2.0);
    trajpoint.time_from_start = duration; 
    traj.points.push_back(trajpoint);

    ros::Rate loop_rate(10);
    int count = 0;
    tf::Vector3 vec;

    while (count<20){
        count++;

        try{
            listener.lookupTransform("/base_link","/kinect_link",ros::Time(0),transform_kinect);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        try{
            listener.lookupTransform("/base_link","/ee_link",ros::Time(0),transform);
            listener.lookupTransform("/base_link","/ee_link",ros::Time(0),start_transform);
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
    node.param("timeout",timeout, 1.125);
    node.param("urdf_param",urdf_param,std::string("/robot_description"));

    TRAC_IK::TRAC_IK tracik_solver(chain_start,chain_end, urdf_param,timeout,eps);

    KDL::Chain chain;
    KDL::JntArray ll,ul;// lower and uper joint limits
    //for (uint tt = 0;tt<ll.data.size();tt++){
    //    ll(tt) = -3;
    //    ul(tt) =3;
    //    ROS_INFO("tt = %d\n",tt);
    //} 
    //ll(0) = -3; ul(0) = 0;
    //ll(1) = -3; ul(1) = 0;
    //ll(2) = 0; ul(2) = 0.75*3;
    //ll(3) = -3; ul(3) = 0;
    //ll(4) = -3; ul(4) = 0;
    //ll(5) = -3; ul(5) = 3;
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
    while(ros::ok()){
        /* get the transform between the base link and the eelink */ 
        try{
            listener.lookupTransform("/base_link","/ee_link",ros::Time(0),transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        /* move the robot according to the status now */
        if(move_to_start_pos){
            start_transform.setOrigin(robot_startpoint);  
            tf::transformTFToKDL(start_transform,end_effector_pose);
            rc = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
            if (rc<0){
                ROS_INFO("no luck no found");
                return -1; 
            }
            ROS_INFO("already get ik solutions");
            ros::Duration duration(1);
            trajpoint.time_from_start = duration; 

            trajpoint.positions[0] = (result(0));
            trajpoint.positions[1] = (result(1));
            trajpoint.positions[2] = (result(2));
            trajpoint.positions[3] = (result(3));
            trajpoint.positions[4] = (result(4));
            trajpoint.positions[5] = (result(5));

            traj.points[0] = trajpoint;

            cmdpub.publish(traj);

            move_to_start_pos = false;
        }
        else{
         
            tf::Vector3 next_step_move;
            next_step_move = computeMotion(handle_leftpoint, handle_rightpoint, vf_handle_rot_dir, move_vec, transform, 0.0 );
            //ROS_INFO("next_step_move: %f,%f,%f",next_step_move.x(),next_step_move.y(),next_step_move.z());
            //transform.setOrigin(transform.getOrigin());
            transform.setOrigin(transform.getOrigin()+next_step_move);
            tf::transformTFToKDL(transform,end_effector_pose);
            rc = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
            if (rc<0){
                ROS_INFO("no luck no found");
                return -1; 
            }

            ros::Duration duration(2.0);
            trajpoint.time_from_start = duration; 
            trajpoint.positions[0] = (result(0));
            trajpoint.positions[1] = (result(1));
            trajpoint.positions[2] = (result(2));
            trajpoint.positions[3] = (result(3));
            trajpoint.positions[4] = (result(4));
            trajpoint.positions[5] = (result(5));

            traj.points[0] = trajpoint;

            cmdpub.publish(traj);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    //transform.setOrigin(transform.getOrigin() + transform.getOrigin().normalized() * (-0.2));
    return 0;
}

tf::Vector3 computeMotion(tf::Vector3 leftvec,tf::Vector3 rightvec,int rot_d, tf::Vector3 move_vec,tf::StampedTransform tran, 
        double tipforce){
    /* we want the robot to be driving by the difference of the applied force 
     * by the stroke gesture and the actural tool tip force
     * the applied force is the norm of the arrow drawn on the screen */
    //ROS_INFO("move_vec: %f\t %f\t %f\t",move_vec.x(),move_vec.y(),move_vec.z());
    tf::Vector3 rot_axis(0,1,0);// all the point are now in a plane perpendicular to y axis
    tf::Vector3 result_vec;//for the result vector
    double diff_move, diff_fixture;
    double eps = 1e-4;
    double k_vec_to_force; // the coeffcience that convert the stroke gesture to force
    k_vec_to_force = MaxForce / 100; // here 100 means the max stroke arrow length we can draw;
    //k_vec_to_force =1; // here 100 means the max stroke arrow length we can draw;
    //diff_f = move_vec.length() * k_vec_to_force - tipforce; 

    /* TODO
     * make sure all the points used are with respect to the same coordinates */
    leftvec.setY(0.0);
    rightvec.setY(0.0);

    ROS_INFO("leftvec: %f,%f \n rightvec: %f,%f",leftvec.x(),leftvec.z(),rightvec.x(),rightvec.z());
    ROS_INFO("rotation_direction: %d", rot_d);
    /* for further computation we need to know the angle that the handle already rotates 
     * the comupation BELOW asume that the points we used are with respect to the robot BASE
     * coordinate*/
    tf::Vector3 rot_center; //rotation center of the handle 
    double orth_angle; // used to find an axis that is perpendicular to the start_vec
    double delta_angle; // used to move the robot forward or backward a little step
    if (rot_d ==1){// two different rotate directions
        rot_center = tf::Vector3(leftvec);  
        orth_angle = -3.141593/2;
        delta_angle = -0.5*3.14159265/180;
    } else{
        rot_center = tf::Vector3(rightvec); 
        orth_angle = 3.141593/2;
        delta_angle = 0.5*3.14159265/180;
    }

    tf::Vector3 robot_start_vec; //start vector of robot position. the angle of ratation 
                                    //are compute with respect to
    robot_start_vec = robot_startpoint - rot_center;
    robot_start_vec.setY(0.0);

    tf::Vector3 robot_cur_vec;//the current position of the robot
    robot_cur_vec = tran.getOrigin() - rot_center;
    robot_cur_vec.setY(0.0);

    tf::Vector3 robot_orth_vec;//orthorgnal vector to the current vector
    robot_orth_vec = robot_start_vec.rotate(rot_axis,orth_angle);

    ROS_INFO("robot_start_vec: %f,%f,%f",robot_start_vec.x(),robot_start_vec.y(),robot_start_vec.z());
    ROS_INFO("robot_cur_vec: %f,%f,%f",robot_cur_vec.x(),robot_cur_vec.y(),robot_cur_vec.z());
    ROS_INFO("robot_orth_vec: %f,%f,%f",robot_orth_vec.x(),robot_orth_vec.y(),robot_orth_vec.z());
    /* now that we have two orthorganal vectors
     * we progect the stroke arrow on each one
     * one progected vector is used to define the whether move the robot
     * the other define wether adjust the fixture */
    double proj_move,proj_fixture;
    // may need a test that .normalize() doesn't change the orignial vector;
    proj_move = k_vec_to_force * move_vec.dot(robot_orth_vec.normalize());//projection along orth_vec; proj_move is a number
    proj_fixture = k_vec_to_force * move_vec.dot(robot_cur_vec.normalize());//projection along cur_vec; proj_fixture is a number

    ROS_INFO("proj_move: %f\t proj_fixture: %f",proj_move,proj_fixture);
    /* send different commands according to the difference between 
     * the length of the stroke gesture and the actual force on the tool tip
     * We can do some tests here without the fixture updating module
     * to make sure the command publishing module is correct. */
    diff_move = proj_move + tipforce; //make sure that tipforce is negative because of 
                                        // its direction
    ROS_INFO("diff_move:%f", diff_move); 
    if (diff_move >eps){ //robot move
        result_vec = robot_cur_vec.rotate(rot_axis,delta_angle)-robot_cur_vec;          
    }else if (diff_move < (-eps)){
        result_vec = robot_cur_vec.rotate(rot_axis,delta_angle)-robot_cur_vec; 
    }else{
        result_vec = robot_cur_vec-robot_cur_vec; 
    } 
    return result_vec;
}
