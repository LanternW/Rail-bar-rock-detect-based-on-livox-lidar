#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <tf/transform_listener.h>
using namespace std;
using namespace Eigen;

#define FALSE 0
#define TRUE  1

#define IMG_LENGTH 1920
#define IMG_HEIGHT 1080
// using namespace cv;

ros::Subscriber livox_lidar_sub , pixel_sub;
ros::Subscriber camera_image_sub;
ros::Publisher  ctd_pc_pub;

cv::Mat curr_image;

bool has_img = FALSE;
bool has_boundingbox = FALSE;
bool use_img_color = FALSE;
bool use_tf_inform = TRUE;

int boundingbox_interrupt = 0;

int current_boundingbox_ulx = 0;
int current_boundingbox_uly = 0;
int current_boundingbox_drx = IMG_LENGTH;
int current_boundingbox_dry = IMG_HEIGHT;


Matrix4d Tc_l ;        // coordinate in Tl to Tc.
Matrix3d K    ;        // camera internal matrix
Matrix4d Tmap_cam = Matrix4d::Identity();    // camera_init frame to aft_mapped frame

string cam_topic, lidar_topic;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZRGB>);


//tf::StampedTransform tf_transform;


void initParams(ros::NodeHandle& n)
{
    double *list4_4 = new double[16];
    double *list3_3 = new double[9];
    XmlRpc::XmlRpcValue param_list;

    if(!n.getParam("/pc_measure/use_img_color", use_img_color)){ ROS_ERROR("Failed to get parameter from server."); }
    //if(!n.getParam("/pc_measure/use_tf_inform", use_tf_inform)){ ROS_ERROR("Failed to get parameter from server."); }
    if(!n.getParam("/pc_measure/cam_topic", cam_topic)){ ROS_ERROR("Failed to get parameter from server."); }
    if(!n.getParam("/pc_measure/lidar_topic", lidar_topic)){ ROS_ERROR("Failed to get parameter from server."); }

    if(!n.getParam("/pc_measure/tc_l", param_list)){ ROS_ERROR("Failed to get parameter from server."); }
    for (size_t i = 0; i < param_list.size(); ++i) 
    {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            list4_4[i] = (double(tmp_value));
    }

    if(!n.getParam("/pc_measure/camera_internal_matrix", param_list)){ ROS_ERROR("Failed to get parameter from server."); }
    for (size_t i = 0; i < param_list.size(); ++i) 
    {
        XmlRpc::XmlRpcValue tmp_value = param_list[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            list3_3[i] = (double(tmp_value));
    }
    
    Tc_l = Map<Matrix4d>(list4_4).transpose();
    K    = Map<Matrix3d>(list3_3).transpose();

    cout << "load params successfully as: \n";
    cout << "Tc_l : " << Tc_l << "\n\n K : " << K << "\n\n use img color: " << use_img_color << endl;
    cout << "cam topic : " << cam_topic << "\nlidar topic : " << lidar_topic  << endl;
    
    ROS_INFO("\n====pc_measure_node init====\n");
}
/*
void updateTF()
{
    Eigen::Vector3d t(
    tf_transform.getOrigin().getX(),
    tf_transform.getOrigin().getY(), 
    tf_transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d   R  = (rot_z_btol.toRotationMatrix() * rot_y_btol.toRotationMatrix() * rot_x_btol.toRotationMatrix());
    Tmap_cam.block<3,3>(0,0) = R;
    Tmap_cam.block<3,1>(0,3) = t;
}
*/
void pushBackPoint(Vector4d pl, int w, int h) {

    if (h < IMG_HEIGHT && h >= 0 && w < IMG_LENGTH && w >= 0){ 

        if (h >= current_boundingbox_uly && h <= current_boundingbox_dry && w >= current_boundingbox_ulx && w <= current_boundingbox_drx){
            Vector4d point_aftmapped_( pl(0) , pl(1), pl(2),1.0 );
            Vector4d point_camerainit = Tmap_cam * point_aftmapped_;
            pcl::PointXYZRGB point;
            point.x = point_camerainit(0);
            point.y = point_camerainit(1);
            point.z = point_camerainit(2);

            //double size = output_pc -> points.size();
            if (use_img_color){
                cv::Vec3b color = curr_image.at<cv::Vec3b>(h, w);
                point.r = color[0];
                point.g = color[1];
                point.b = color[2];
            }
            else{
                point.r = 255;
                point.g = 255;
                point.b = 0;
            }
            //cout<<output_pc->size()<<" points"<<endl;
            output_pc -> points.push_back(point);
        }
    }
}

void clearPCMap() {
    output_pc -> clear();
}

void imageCallback(const sensor_msgs::Image::ConstPtr& img) {

    boundingbox_interrupt++;
    if(boundingbox_interrupt > 5){
        cout << "bounding box lost! " << boundingbox_interrupt <<endl;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    { 
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8); 
    } 
    catch(cv_bridge::Exception& e)
    { 
        ROS_ERROR("cv_bridge exception: %s", e.what()); 
        return; 
    } 
    curr_image = cv_ptr -> image ; 
    has_img = TRUE; 

    if(boundingbox_interrupt > 5){
       return;
    }
    sensor_msgs::PointCloud2 output_pc_ros;
    int points_count = output_pc -> size();
    cout << points_count << "points published." << endl;
    pcl::toROSMsg( *output_pc ,output_pc_ros);

    clearPCMap();
    output_pc_ros.header.frame_id = "camera_init";
    ctd_pc_pub.publish(output_pc_ros);

} 


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_now){

    if (has_boundingbox == FALSE || has_img == FALSE){return ;}
    if (output_pc -> size() > 40000){
        ROS_INFO("Too many cache points, check camera.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg( *pc_now, laserCloudIn );

    Vector4d pl, pc;
    Vector3d pc_, v;
    int w,h;
    for(int i = 0; i < laserCloudIn.points.size() ; i++)
    {

        pl(0) = laserCloudIn.points[i].x;
        pl(1) = laserCloudIn.points[i].y;
        pl(2) = laserCloudIn.points[i].z;
        pl(3) = 1;
        pc = Tc_l * pl;
        pc_(0) = pc(0) / pc(2);
        pc_(1) = pc(1) / pc(2);
        pc_(2) = 1;

        v = K * pc_;
        w = v(0);
        h = v(1);

        pushBackPoint( pl, w,h );
    }
}

void pixelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    
    int w0 = msg -> linear.x;
    int h0 = msg -> linear.y;
    int w1 = msg -> angular.x;
    int h1 = msg -> angular.y;


    if ( w0 < 0 || w0 >= IMG_LENGTH || h0 < 0 || h0 >= IMG_HEIGHT ){return;}
    if ( w1 < 0 || w1 >= IMG_LENGTH || h1 < 0 || h1 >= IMG_HEIGHT ){return;}

    boundingbox_interrupt = 0;
    has_boundingbox = TRUE;

    current_boundingbox_ulx = w0;
    current_boundingbox_uly = h0;
    current_boundingbox_drx = w1;
    current_boundingbox_dry = h1;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_measure");
    ros::NodeHandle nh;

    //tf::TransformListener tf_listener; 

    initParams(nh);
    livox_lidar_sub =
        nh.subscribe(lidar_topic , 1000, pointCloudCallback);
    
    camera_image_sub =
        nh.subscribe(cam_topic , 1000, imageCallback);

    pixel_sub =
        nh.subscribe("/image_pixel" , 10, pixelCallback);

    ctd_pc_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/roi_point_cloud", 1000);

/*
    while(ros::ok())
    {
		try
		{
			tf_listener.waitForTransform("/camera_init", "/aft_mapped", ros::Time(0), ros::Duration(3.0));
			tf_listener.lookupTransform("/camera_init", "/aft_mapped", ros::Time(0), tf_transform);
            //if (use_tf_inform){
            //updateTF();
            //    }
		}
        catch(exception e)
        {
            cout << "tf lisening error" << endl;
        }
        ros::spinOnce();
    }
    */
    ros::spin();
    return 0;
}