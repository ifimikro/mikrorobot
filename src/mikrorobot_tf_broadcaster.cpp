#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mikrorobot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    while(n.ok()){

        tf::StampedTransform world_to_camera_transform;
        tf::StampedTransform camera_to_base_transform;
        try{
            listener.lookupTransform("ORB_SLAM/World", "ORB_SLAM/Camera",
                                     ros::Time(0), world_to_camera_transform);
            listener.lookupTransform("camera", "base_link",
                                     ros::Time(0), camera_to_base_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform().getIdentity(),
                ros::Time::now(),"/ORB_SLAM/World", "map"));
        broadcaster.sendTransform(
            tf::StampedTransform(
                world_to_camera_transform,
                ros::Time::now(),"map", "odom"));
        broadcaster.sendTransform(
            tf::StampedTransform(
                camera_to_base_transform,
                ros::Time::now(),"odom", "base_link"));
        r.sleep();
    }
}
