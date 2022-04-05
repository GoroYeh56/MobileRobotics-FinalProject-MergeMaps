#include <ros/ros.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <cloud_to_grid/Grid_Tool.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <dynamic_reconfigure/server.h>
// #include <cloud_to_grid/cloud_to_gridConfig.h>


#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

using namespace std;

void TimerCallBack(const ros::TimerEvent&);
// void CloudCallBack(const sensor_msgs::PointCloud2ConstPtr ros_cloud);



ros::Publisher final_map_pub;
ros::Subscriber map1_sub;
ros::Subscriber map2_sub;

nav_msgs::OccupancyGrid grid1;
nav_msgs::OccupancyGrid grid2;
nav_msgs::OccupancyGrid grid;
int map_counter=0;

// Store /map1 data to grid1
void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    cout<<"map1 callback\n";
    grid1.header = msg->header;
    grid1.info = msg->info;
    grid1.data = msg->data;
}

// Store /map2 data to grid2
void map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    cout<<"map2 callback\n";
    grid2.header = msg->header;
    grid2.info = msg->info;
    // ROS_INFO("Got map %d %d", info.width, info.height);
    // Map map(info.width, info.height);
    grid2.data = msg->data;
}


/*
    Subscibe:  /map1
    Subscribe: /map2

    Publish:   /final_map

    nav_msgs/OccupancyGrid.msg

    std_msgs/Header header
    nav_msgs/MapMetaData info
    int8[] data

*/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_map_node");
    ros::start();

    // mygrid=new MyTool::MyGrid();
    ros::NodeHandle nh;
    map1_sub = nh.subscribe("map1", 1, map1Callback);//need to change by yourself
    map2_sub = nh.subscribe("map2", 1, map2Callback);
    final_map_pub= nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

    // Publish merged_map her
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), TimerCallBack);

    // dynamic_reconfigure::Server<cloud_to_grid::cloud_to_gridConfig> server;
    // dynamic_reconfigure::Server<cloud_to_grid::cloud_to_gridConfig>::CallbackType f;
    // f = boost::bind(&callback, _1, _2);
    // server.setCallback(f);

    // Initialized grid.data


    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    // ros::spin();
    return 0;
}

// Q: when to updat mapData?
void TimerCallBack(const ros::TimerEvent&)
{
    cout<<"TimerCallBack\n";
    if(!grid1.data.empty() && !grid2.data.empty()){
        // grid.header.seq = map_counter++;
        // grid.header.frame_id = "odom";
        // grid.info.origin.position.z = 0;
        // grid.info.origin.orientation.w = 1;
        // grid.info.origin.orientation.x = 0;
        // grid.info.origin.orientation.y = 0;
        // grid.info.origin.orientation.z = 0;

        // grid.header.stamp.sec = ros::Time::now().sec;
        // grid.header.stamp.nsec = ros::Time::now().nsec;
        // grid.info.map_load_time = ros::Time::now();

        grid.header=grid1.header;
        grid.info = grid1.info;
        // grid.info.resolution = mapData.resolution;
        // grid.info.width = mapData.width;
        // grid.info.height = mapData.height;
        // grid.info.origin.position.x = mapData.map_length/2*-1;  //minx
        // grid.info.origin.position.y = mapData.map_length/2*-1;  //miny
        // grid.data = mapData.data;

        // Average grid1 & grid2 here!
        cout<<"grid1.data.size() "<<grid1.data.size()<<endl;
        cout<<"grid2.data.size() "<<grid2.data.size()<<endl;
        int mapSize = min(grid1.data.size(), grid2.data.size());
        // cout<< (grid.data.size()) <<endl;

        grid.data.resize(mapSize);
        for(int i=0; i< mapSize; ++i){
            grid.data[i] = (grid1.data[i] + grid2.data[i])/2 ;
        }

        final_map_pub.publish(grid);
        std::cout<<"Pub grid!!"<< std::endl;

    }
}




