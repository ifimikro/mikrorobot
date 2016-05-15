#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mikrobot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while(n.ok()){
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(-0.06, 0.0, -0.1)),
                ros::Time::now(),"ORB_SLAM/Camera", "base_link"));
        r.sleep();
    }
}
