#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>

class LaserClusterProcessor
{
public:
    LaserClusterProcessor()
    {
        // Subscribe to the laser scan topic
        laser_sub_ = nh_.subscribe("/scan", 1, &LaserClusterProcessor::laserCallback, this);

        // Publish midpoints
        midpoint_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/midpoints", 1);

        // Publish transformed scan
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/transformed_scan", 1);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Convert LaserScan to PointCloud with odom as the reference frame
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i]))
                continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            pcl::PointXYZ point;
            point.x = range * cos(angle); //-1.7); // x-coordinate in odom frame
            point.y = range * sin(angle); //-1.7); // y-coordinate in odom frame
            point.z = 0;
            cloud.points.push_back(point);
        }

        // Process clusters
        std::vector<std::vector<pcl::PointXYZ>> clusters = clusterPoints(cloud);

        pcl::PointCloud<pcl::PointXYZ> midpoints_cloud;

        // Compute midpoints between clusters
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            for (size_t j = i + 1; j < clusters.size(); ++j)
            {
                float distance = calculateClusterDistance(clusters[i], clusters[j]);

                if (distance > 0.25 && distance < 0.75)
                {
                    pcl::PointXYZ midpoint = calculateClusterMidpoint(clusters[i], clusters[j]);
                    midpoints_cloud.points.push_back(midpoint);
                }
            }
        }

        if (!midpoints_cloud.points.empty())
        {
            sensor_msgs::PointCloud2 midpoint_msg;
            pcl::toROSMsg(midpoints_cloud, midpoint_msg);
            midpoint_msg.header.frame_id = "base_link";
            midpoint_msg.header.stamp = msg->header.stamp;
            midpoint_pub_.publish(midpoint_msg);
        }

        sensor_msgs::LaserScan transformed_scan = *msg;
        transformed_scan.header.frame_id = "base_link";
        scan_pub_.publish(transformed_scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher midpoint_pub_;
    ros::Publisher scan_pub_;

    // Cluster processing functions
    std::vector<std::vector<pcl::PointXYZ>> clusterPoints(const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        std::vector<std::vector<pcl::PointXYZ>> clusters;
        std::vector<pcl::PointXYZ> current_cluster;

        for (size_t i = 1; i < cloud.points.size(); ++i)
        {
            float distance = sqrt(pow(cloud.points[i].x - cloud.points[i - 1].x, 2) +
                                  pow(cloud.points[i].y - cloud.points[i - 1].y, 2));

            if (distance < 0.20)
            {
                current_cluster.push_back(cloud.points[i - 1]);
            }
            else
            {
                if (!current_cluster.empty())
                {
                    current_cluster.push_back(cloud.points[i - 1]);
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
            }
        }

        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        return clusters;
    }

    float calculateClusterDistance(const std::vector<pcl::PointXYZ> &cluster1, const std::vector<pcl::PointXYZ> &cluster2)
    {
        pcl::PointXYZ centroid1 = calculateClusterCentroid(cluster1);
        pcl::PointXYZ centroid2 = calculateClusterCentroid(cluster2);

        return sqrt(pow(centroid2.x - centroid1.x, 2) + pow(centroid2.y - centroid1.y, 2));
    }

    pcl::PointXYZ calculateClusterCentroid(const std::vector<pcl::PointXYZ> &cluster)
    {
        float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto &point : cluster)
        {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }
        size_t num_points = cluster.size();

        return pcl::PointXYZ(sum_x / num_points, sum_y / num_points, sum_z / num_points);
    }

    pcl::PointXYZ calculateClusterMidpoint(const std::vector<pcl::PointXYZ> &cluster1, const std::vector<pcl::PointXYZ> &cluster2)
    {
        pcl::PointXYZ centroid1 = calculateClusterCentroid(cluster1);
        pcl::PointXYZ centroid2 = calculateClusterCentroid(cluster2);

        return pcl::PointXYZ((centroid1.x + centroid2.x) / 2.0,
                             (centroid1.y + centroid2.y) / 2.0,
                             (centroid1.z + centroid2.z) / 2.0);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_cluster_processor");
    LaserClusterProcessor processor;
    ros::spin();
    return 0;
}
