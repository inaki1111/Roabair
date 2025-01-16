#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ScanToMapTransformer {
public:
    ScanToMapTransformer() {
        // Initialize robot position at (0, 0, 0) in the odom frame
        robot_position_.x = 0.0;
        robot_position_.y = 0.0;
        robot_position_.theta = 0.0;

        // Subscribe to laser scan topic
        scan_sub_ = nh_.subscribe("/scan", 10, &ScanToMapTransformer::scanCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    struct Position {
        double x;
        double y;
        double theta; // Orientation in radians
    } robot_position_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // Broadcast the transform from odom to base_link (robot's position in odom)
        geometry_msgs::TransformStamped odom_to_base_link;
        odom_to_base_link.header.stamp = ros::Time::now();
        odom_to_base_link.header.frame_id = "odom";
        odom_to_base_link.child_frame_id = "laser";
        odom_to_base_link.transform.translation.x = robot_position_.x;
        odom_to_base_link.transform.translation.y = robot_position_.y;
        odom_to_base_link.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_position_.theta);
        odom_to_base_link.transform.rotation.x = q.x();
        odom_to_base_link.transform.rotation.y = q.y();
        odom_to_base_link.transform.rotation.z = q.z();
        odom_to_base_link.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(odom_to_base_link);

        // Transform the laser scan from base_link to map frame
        try {
            geometry_msgs::TransformStamped map_to_odom = tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
            geometry_msgs::TransformStamped base_link_to_map;
            tf2::doTransform(odom_to_base_link, base_link_to_map, map_to_odom);

            ROS_INFO_STREAM("Laser scan transformed from base_link to map:");
            ROS_INFO_STREAM("Position: [" << base_link_to_map.transform.translation.x << ", "
                            << base_link_to_map.transform.translation.y << ", "
                            << base_link_to_map.transform.translation.z << "]");
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform base_link to map: %s", ex.what());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_to_map");
    ScanToMapTransformer transformer;
    ros::spin();
    return 0;
}
