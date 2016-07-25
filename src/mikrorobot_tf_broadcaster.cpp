#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mikrorobot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while(n.ok()){

        broadcaster.sendTransform(tf::StampedTransform(tf::Transform().getIdentity(), ros::Time::now(), "base_footprint", "base_link"));
         broadcaster.sendTransform(tf::StampedTransform(tf::Transform().getIdentity(), ros::Time::now(), "world", "map"));
          broadcaster.sendTransform(tf::StampedTransform(tf::Transform().getIdentity(), ros::Time::now(), "odom", "odom_combined"));


        r.sleep();
    }
}
