#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>

#include <iostream>
#include <stdlib.h>
#include <math.h>

#define PI 3.141592653589
using namespace std;
ros::Subscriber livox_lidar_sub;
ros::Publisher  height_pc_pub ,filtered_pc_pub ,rotated_pc_pub;
pcl::PointXYZ highest_point;

double max_distance_multi = 0;
int total_frame = 0;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_now) {

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_raw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_raw_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg( *pc_now, *pc_raw );

    /*
    Eigen::Matrix4f transform_pre = Eigen::Matrix4f::Identity();
    Eigen::AngleAxisf Rotation_vector_pre(PI/2, Eigen::Vector3f(0,1,0));
    Eigen::Matrix3f rotation_matrix_pre = Rotation_vector_pre.matrix();
    transform_pre.block<3,3>(0,0) = rotation_matrix_pre;
    pcl::transformPointCloud(*pc_raw_pre , *pc_raw , transform_pre);*/

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setMaxIterations(500);
    seg.setInputCloud (pc_raw);
    seg.segment (*inliers, *coefficients);

    // 把平面内点提取到一个新的点云中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> ex ;
    ex.setInputCloud(pc_raw);
    ex.setIndices(inliers);
    ex.filter(*cloud_plane); 

    //统计滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (pc_raw);                           //设置待滤波的点云
    sor.setMeanK (10);                               //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (2.0);                      //设置判断是否为离群点的阀值
    sor.filter (*cloud_filtered);                    //存储

    float A = coefficients -> values[0];
    float B = coefficients -> values[1];
    float C = coefficients -> values[2];
    float D = coefficients -> values[3];
    double d = sqrt(A*A + B*B + C*C);
    double theta = acos(fabs(C));
    double maxdistance = 0 , index = 0;
    double distance = 0;

    cout<<"theta = "<<theta<<endl;
    for(int i = 0 ; i < cloud_filtered -> points.size() ; i++) {

       // if(cloud_filtered -> points[i].z < 0.05){continue;}
        distance = abs(A*cloud_filtered -> points[i].x + B*cloud_filtered -> points[i].y + C*cloud_filtered -> points[i].z + D) / d;
        if (distance > maxdistance) { maxdistance = distance ; index = i;}
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::AngleAxisf Rotation_vector(theta , Eigen::Vector3f(0,1,0));
    Eigen::Matrix3f rotation_matrix = Rotation_vector.matrix();
    transform.block<3,3>(0,0) = rotation_matrix;
    pcl::transformPointCloud(*cloud_filtered , *cloud_rotated , transform);

    sensor_msgs::PointCloud2 output_pc_rotated, output_pc_filtered;
    pcl::toROSMsg(*cloud_rotated , output_pc_rotated);
    pcl::toROSMsg(*cloud_filtered , output_pc_filtered);
    output_pc_rotated.header.frame_id = "livox_frame";
    output_pc_filtered.header.frame_id = "livox_frame";
    filtered_pc_pub.publish(output_pc_filtered);
    rotated_pc_pub.publish(output_pc_rotated);

    if(maxdistance < 0.2) {return ;}

    if(max_distance_multi < maxdistance){
        max_distance_multi = maxdistance;
        highest_point = cloud_filtered -> points[index];
    }
    total_frame ++;

    if (total_frame >= 5) {

        
        sensor_msgs::PointCloud2 output_pc ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        output_cloud -> push_back(highest_point);

        pcl::toROSMsg(*output_cloud , output_pc);
        output_pc.header.frame_id = "livox_frame";

        height_pc_pub.publish(output_pc);

        double dx = -highest_point.x;
        double dy = -highest_point.y;

        max_distance_multi = 0;
        total_frame = 0;

        cout<<"\npublished!  dx="<<dx <<" | dy="<<dy <<endl;
    }




}

//驴肉火烧
int main(int argc, char** argv)
{
    ros::init(argc, argv, "highest_exactor_node");
    ros::NodeHandle nh;

    livox_lidar_sub =
        nh.subscribe("/livox/lidar" , 1000, pointCloudCallback);

    height_pc_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/highest_point_cloud", 1000);

    filtered_pc_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1000); 
    
    rotated_pc_pub = 
        nh.advertise<sensor_msgs::PointCloud2>("/rotated_point_cloud", 1000); 

    ros::spin();
    return 0;
}

