
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

using namespace std;

class datasaver{
public:
    datasaver(ros::NodeHandle &_n):nh(_n){
        semap_sub = nh.subscribe("/semantic_map", 1, &datasaver::semanticMapCB, this);
        elemap_sub = nh.subscribe("/elevation_map", 1, &datasaver::elevationMapCB, this);
        img_sub = nh.subscribe("image", 1, &datasaver::imageCB, this);
        pcl_sub = nh.subscribe("pointcloud", 1, &datasaver::pclCB, this);
        nh.param("savedata/writepath", writepath, string("data"));
//        pclfrommsg = new pcl::PointCloud<pcl::PointXYZ>;
    }
    ~datasaver(){cv::destroyWindow("save_image");}
private:
    ros::NodeHandle nh;
    ros::Subscriber semap_sub, elemap_sub, img_sub, pcl_sub;
    grid_map::GridMap semantic_GM, elevation_GM;
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZ> pclfrommsg;
    string writepath;

    void savedata(cv::Mat img){
        //save image
        cv::imwrite(writepath+"image.jpg", img);
        ROS_INFO("save image done.");

        //save elevation map matrix
        if(elevation_GM.exists("elevation")){
            //Eigen::MatrixXf eigmat = elevation_GM.get("elevation");
            //ofstream output_file0(writepath+"elemap.txt", ios::out | ios::trunc);
            //output_file0.close();
            elevation_GM.add("res");
            cv::Mat elevation;
            //writepath = "/home/yimo/catkin_ws_isolated/src/sele_path_planner/data/";
            grid_map::GridMapCvConverter::toImage<float, 1>(elevation_GM, "elevation", CV_32FC1, -2.0, 2.0, elevation);
            cv::imwrite(writepath+"elevation.png", elevation);
            ofstream ofs(writepath+"elemap.txt");
            std::cout<<elevation<<endl;
            for(int row = 0;row<elevation.rows;row++){
                for(int col = 0;col<elevation.cols;col++){
                    ofs<<(elevation.ptr<float>(elevation.rows-1 - row)[col])<<" ";
                }
                ofs<<"\n";
            }

//            cv::Mat res_road;
//            grid_map::GridMapCvConverter::toImage<float, 1>(semantic_GM, "prob", CV_32FC1, 0.0, 1.0, res_road);
//            cv::imwrite(writepath+"prob.png", res_road);
//            ofstream ofs_res_road(writepath+"prob.txt");
//            for(int row = 0;row<elevation.rows;row++){
//                for(int col = 0;col<elevation.cols;col++){
//                    ofs_res_road<<(elevation.ptr<float>(row)[col])<<" ";
//                }
//                ofs_res_road<<"\n";
//            }

            for (grid_map::GridMapIterator iterator(elevation_GM); !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
                elevation_GM.at("res",*iterator) = elevation.ptr<float>(imageIndex(0))[imageIndex(1)];
            }
            //ofstream ofs(writepath+"elemap.txt");
            //for (grid_map::GridMapIterator iterator(elevation_GM); !iterator.isPastEnd(); ++iterator) {
            //    ofs<< elevation_GM.at("elevation",*iterator)<<endl;
            //}

            //ofs<<elevation_GM["res"];
            ofs.close();
            //ofs_res_road.close();
            ROS_INFO("save elevation done.");
        }
        else{
            ROS_INFO("no elevation.");
        }

        //save point cloud
        if(pclfrommsg.size() < 1){
            ROS_INFO("no point cloud.");
        }
        else{
            ofstream ofs(writepath+"pcl.txt");
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(pclfrommsg, pclfrommsg, indices);
            for(int i=0; i<pclfrommsg.size(); ++i){
                ofs<<pclfrommsg.points[i].x<<" "<<pclfrommsg.points[i].y<<" "<<pclfrommsg.points[i].z<<endl;
            }
            ROS_INFO("save point cloud done.");
        }
    }

    void elevationMapCB(const grid_map_msgs::GridMapConstPtr &msg){
        grid_map::GridMapRosConverter::fromMessage(*msg, elevation_GM);
    }
    void semanticMapCB(const grid_map_msgs::GridMapConstPtr &msg){
        grid_map::GridMapRosConverter::fromMessage(*msg, semantic_GM);
    }
    void pclCB(const sensor_msgs::PointCloud2ConstPtr &msg){
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, pclfrommsg);
    }
    void imageCB(const sensor_msgs::CompressedImageConstPtr &msg){
        cv::Mat imginput = cv::imdecode(cv::Mat(msg->data),1);
//        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        cv::imshow("save_image", imginput);
        int keyint = cv::waitKey(5);
        if(keyint != -1){
            savedata(imginput);
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv,"se_planner");
    ros::NodeHandle nh;
    datasaver dts(nh);
    ros::spin();
    return 0;
}
