#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;
    
    tf::TransformListener listener;

    ros::Rate(10);
    while(node.ok()){
        tf::StampedTransform transform;
        tf::Vector3 vec;
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
}
