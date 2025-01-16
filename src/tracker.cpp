#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

class MidpointsToCmdVel {
public:
    MidpointsToCmdVel() {
        // Suscripciones y publicaciones
        pointcloud_sub_ = nh_.subscribe("midpoints", 1, &MidpointsToCmdVel::pointCloudCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);   
         }


private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher cmd_vel_pub_;

    double robot_yaw_ = 0.0;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convertir PointCloud2 a PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        double min_distance = std::numeric_limits<double>::max();
        double target_angle = 0.0;

        // Buscar el punto más cercano dentro del rango
        for (const auto& point : cloud.points) {
            double distance = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance < min_distance && distance < 1) {
                min_distance = distance;
                target_angle = std::atan2(point.y, point.x);
            }
        }

        // Si hay un punto cercano, publicar comando para rotar
        if (min_distance < 2) {
            geometry_msgs::Twist cmd;
            double angle_error = target_angle - robot_yaw_;

            // Normalizar el error de ángulo entre -pi y pi
            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;

            ROS_INFO("robot yaw: %f",robot_yaw_);    
            ROS_INFO("angle error: %f",angle_error);

            if (abs(angle_error) < 0.3){
                    cmd.angular.z = 0; // Ganancia proporcional para la rotación
                    if (min_distance < 0.2) {
                        cmd.linear.x = 0; // Solo rotamos
                    }
                    else{
                        cmd.linear.x = 0.3; // Solo rotamos    
                    }
                    
            }
            else{
            cmd.angular.z = 0.4 * angle_error; // Ganancia proporcional para la rotación
            cmd.linear.x = 0.0; // Solo rotamos
            ROS_INFO("turning");
            }

            cmd_vel_pub_.publish(cmd);
            
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker");
    MidpointsToCmdVel node;
    ros::spin();
    return 0;
}
