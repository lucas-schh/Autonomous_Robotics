
#include <vector>
#include <string>
#include <map>
#include <list>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include "wpa_cli/Scan.h"
#include "wpa_cli/Network.h"

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

class OccupancyGridPlanner {
    // public:
    //     cv::Mat_<cv::Vec3b> getOg_rgb_();
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Subscriber og_signal_sub_;
        ros::Subscriber path_current;
        ros::Subscriber exploration_sub;
        ros::Publisher path_pub_;
        ros::Publisher target_pub_;
        tf::TransformListener listener_;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_, cropped_og_signal;
        cv::Mat_<int>og_signal_count;
        cv::Mat_<double> og_signal, og_signal_sum;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_,og_signal_rgb_,og_signal_rgb_marked_;
        cv::Point3i og_center_;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        unsigned int neighbourhood_;
        bool ready;
        bool debug;
        double radius;
        bool create_signal_map;
        bool start_exploration;
        std::vector<cv::Point> frontier_og;
        std::vector<cv::Point> frontier_signal;

        unsigned int maxx,maxy,minx,miny;

        typedef std::multimap<float, cv::Point3i> Heap;

        double heuristic(const cv::Point3i& current, const cv::Point3i& goal) {
            return hypot(goal.x - current.x, goal.y - current.y);
        }

        cv::Point from3Dto2D(const cv::Point3i & P_3D){
            return cv::Point(P_3D.x,P_3D.y);
        }

        // Callback for Occupancy Grids
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point3i(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution,0);

            // Some variables to select the useful bounding box 
            maxx=0;
            minx=msg->info.width;
            maxy=0;
            miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    if(og_(j,i)!=UNKNOWN){
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            double erosion_size=ceil(radius/info_.resolution);
            cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT ,
                                    cv::Size( 2*erosion_size+3, 2*erosion_size+3 ),
                                    cv::Point( erosion_size, erosion_size ) );
            cv::erode(og_,og_,element);
            if (!ready) {
                ready = true;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            cv::cvtColor(og_, og_rgb_, cv::COLOR_GRAY2RGB);
            frontier_og.clear();
            //fill frontier list
            for (unsigned int i=0;i<msg->info.height;i++) {
                for (unsigned int j=0;j<msg->info.width;j++) {
                    if(og_(i,j)==FREE && (og_(i+1,j)==UNKNOWN || og_(i+1,j+1)==UNKNOWN || og_(i,j+1)==UNKNOWN || og_(i-1,j+1)==UNKNOWN || og_(i-1,j)==UNKNOWN || og_(i-1,j-1)==UNKNOWN || og_(i,j-1)==UNKNOWN || og_(i+1,j-1)==UNKNOWN)){
                        cv::Point pos = cv::Point(j,i);
                        frontier_og.push_back(pos);
                        og_rgb_(pos)=cv::Vec3b(255,0,0);
                    }
                }
            }
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og;
                cv::resize(cropped_og_,resized_og,new_size);
                cv::imwrite( "/home/turtlebot/catkin_ws/map/OccGrid.png", resized_og );
            } else {
                cv::imwrite( "/home/turtlebot/catkin_ws/map/OccGrid.png", og_rgb_ );
            }

            // this gets the current pose in transform
            tf::StampedTransform transform;
            listener_.waitForTransform(frame_id_,base_link_,ros::Time::now(),ros::Duration(1.0));
            listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
            cv::Point3i actual_pos;
            if (debug) {
                actual_pos = og_center_;
            } else {
                actual_pos = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)(round(tf::getYaw(transform.getRotation())/(M_PI/4)))%8)
                + og_center_;
            }

            for(int x=actual_pos.x-5;x<actual_pos.x+5;x++){
                for(int y=actual_pos.y-5;y<actual_pos.y+5;y++){
                    cv::Point rect=cv::Point(x,y);
                    og_(rect)=FREE;
                }
            }
        }

        void og_signal_callback(const wpa_cli::Scan & msg) {
            tf::StampedTransform transform;
            if (!ready) {
                return;
            }
            if(create_signal_map){
                og_signal_count=cv::Mat_<int>(og_.size(),0);
                og_signal_sum=cv::Mat_<double>(og_.size(),0.0);
                og_signal=cv::Mat_<double>(og_.size(),1.0);
                create_signal_map=false;
            }
            // this gets the current pose in transform
            listener_.waitForTransform(frame_id_,base_link_,ros::Time::now(),ros::Duration(1.0));
            listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
            cv::Point3i actual_pos;
            if (debug) {
                actual_pos = og_center_;
            } else {
                actual_pos = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)(round(tf::getYaw(transform.getRotation())/(M_PI/4)))%8)
                + og_center_;
            }
            // ROS_INFO("position x = %d",actual_pos.x);
            // ROS_INFO("position y = %d",actual_pos.y);
            for(int x=actual_pos.x-10;x<actual_pos.x+10;x++){
                for(int y=actual_pos.y-10;y<actual_pos.y+10;y++){
                    cv::Point rect=cv::Point(x,y);
                    og_signal_count(rect)+=1;
                    og_signal_sum(rect)+=-msg.networks[0].level/100.0; //valeur max??
                    og_signal(rect)=(og_signal_count(rect)!=0) ? og_signal_sum(rect)/float(og_signal_count(rect)) : 1;
                }
            }
            if (!ready) {
                ready = true;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            cv::Mat_<uint8_t> og_signal_uint = cv::Mat_<uint8_t>(og_signal*255);
            cv::cvtColor(og_signal_uint, og_signal_rgb_, cv::COLOR_GRAY2RGB);
            // fill the frontier
            frontier_signal.clear();
            for (unsigned int i=0;i<og_.rows;i++) {
                for (unsigned int j=0;j<og_.cols;j++) {
                    if(og_(i,j)!=OCCUPIED && og_signal_count(i,j)==1 && (og_signal_count(i+1,j)==0 || og_signal_count(i+1,j+1)==0 || og_signal_count(i,j+1)==0 || og_signal_count(i-1,j+1)==0 || og_signal_count(i-1,j)==0 || og_signal_count(i-1,j-1)==0 || og_signal_count(i,j-1)==0 || og_signal_count(i+1,j-1)==0)){
                        cv::Point pos = cv::Point(j,i);
                        frontier_signal.push_back(pos);
                        og_signal_rgb_(pos)=cv::Vec3b(255,0,0);
                    }
                }
            }
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_signal = cv::Mat_<uint8_t>(og_signal,roi_);
            cv::circle(og_signal_rgb_,from3Dto2D(actual_pos), 10, cv::Scalar(0,0,255));
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og_signal;
                cv::resize(cropped_og_signal,resized_og_signal,new_size);
                cv::imwrite( "/home/turtlebot/catkin_ws/map/OccGrid_signal.png", resized_og_signal );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                cv::imwrite( "/home/turtlebot/catkin_ws/map/OccGrid_signal.png", og_signal_rgb_ );
            }
        }

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point3i & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }

        void path_pixel_callback(const geometry_msgs::PoseStampedConstPtr & msg){
            if(start_exploration){
                geometry_msgs::PoseStamped pose;
                tf::StampedTransform transform;
                pose = *msg;
                if (debug) {
                } else {
                    // This converts target in the grid frame.
                    listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                    listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
                }
                // Now get the current point in grid coordinates.
                cv::Point3i start;
                if (debug) {
                    start = og_center_;
                } else {
                    start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)(round(tf::getYaw(transform.getRotation())/(M_PI/4)))%8)
                        + og_center_;
                }
                ROS_INFO("Planning origin %.2f %.2f %.2f -> %d %d %d",
                        transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()), start.x, start.y, start.z);
                og_rgb_marked_ = og_rgb_.clone();
                og_signal_rgb_marked_ = og_signal_rgb_.clone();
                cv::Point3i target;
                target = cv::Point3i(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
                cv::circle(og_rgb_marked_,from3Dto2D(target), 10, cv::Scalar(0,0,255));
                cv::circle(og_signal_rgb_marked_,from3Dto2D(target), 10, cv::Scalar(0,0,255));
                if (!isInGrid(target)) {
                    ROS_ERROR("Invalid target point (%d %d)",
                            target.x, target.y);
                    return;
                }
                // Only accept target which are FREE in the grid (HW, Step 5).
                if (og_(from3Dto2D(target)) == OCCUPIED) {
                    ROS_ERROR("Invalid target point: occupancy = %d",og_(from3Dto2D(target)));
                    //if target is not right search for a new target point
                    std_msgs::Bool msg;
                    path_current_callback(msg);
                    return;
                }
                cv::circle(og_rgb_marked_,from3Dto2D(start), 10, cv::Scalar(0,255,0));
                cv::circle(og_signal_rgb_marked_,from3Dto2D(start), 10, cv::Scalar(0,255,0));
                if (!isInGrid(start)) {
                    ROS_ERROR("Invalid starting point (%d %d %d)",
                            start.x, start.y, start.z);
                    return;
                }
                ROS_INFO("Starting planning from (%d, %d,%d) to (%d, %d, %d)",start.x,start.y,start.z, target.x, target.y, target.z);
                // Here the Dijskstra algorithm starts 
                // The best distance to the goal computed so far. This is
                // initialised with Not-A-Number. 
                // cv::Mat_<float> cell_value(og_.size(), NAN);
                int dims[3] = {og_.rows,og_.cols,8};
                cv::Mat_<float> cell_value(3,dims,NAN);
                // For each cell we need to store a pointer to the coordinates of
                // its best predecessor. 
                // cv::Mat_<cv::Vec3s> predecessor(og_.size());
                cv::Mat_<cv::Vec3s> predecessor(3,dims);

                // The neighbour of a given cell in relative coordinates. The order
                // is important. If we use 4-connexity, then we can use only the
                // first 4 values of the array. If we use 8-connexity we use the
                // full array.
                std::vector<std::vector<cv::Point3i>> neighbours;
                neighbours={{cv::Point3i(1,0,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //0
                            {cv::Point3i(1,1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //45
                            {cv::Point3i(0,1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //90
                            {cv::Point3i(-1,1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //135
                            {cv::Point3i(-1,0,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //180
                            {cv::Point3i(-1,-1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //225
                            {cv::Point3i(0,-1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}, //270
                            {cv::Point3i(1,-1,0),cv::Point3i(0,0,1),cv::Point3i(0,0,-1)}}; //315
                // Cost of displacement corresponding the neighbours. Diagonal
                // moves are 44% longer.
                std::vector<std::vector<double>> cost;
                cost={{1,2,2}, //0
                    {sqrt(2),2,2}, //45
                    {1,2,2}, //90
                    {sqrt(2),2,2}, //135
                    {1,2,2}, //180
                    {sqrt(2),2,2}, //225
                    {1,2,2}, //270
                    {sqrt(2),2,2}}; //315

                // The core of Dijkstra's Algorithm, a sorted heap, where the first
                // element is always the closer to the start.
                Heap heap;
                heap.insert(Heap::value_type(heuristic(start, target), start));
                while (!heap.empty() && !(heap.begin()->second==target)) {
                    // Select the cell at the top of the heap
                    Heap::iterator hit = heap.begin();
                    // the cell it contains is this_cell
                    cv::Point3i this_cell = hit->second;
                    // and its score is this_cost
                    float this_cost = hit->first;
                    // We can remove it from the heap now.
                    heap.erase(hit);
                    // Now see where we can go from this_cell

                    for (unsigned int i=0;i<neighbourhood_;i++) {
                        cv::Point3i dest = this_cell + neighbours[this_cell.z][i];
                        if (!isInGrid(dest)) {
                            // outside the grid
                            continue;
                        }
                        uint8_t og = og_(from3Dto2D(dest));
                        if (og == OCCUPIED) {
                            // occupied or unknown
                            continue;
                        }
                        dest.z=(dest.z+8)%8;
                        float cv = cell_value(dest.x,dest.y,dest.z);
                        float new_cost = this_cost + cost[this_cell.z][i];
                        if (std::isnan(cv) || (new_cost < cv)) {
                            // found shortest path (or new path), updating the
                            // predecessor and the value of the cell
                            predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                            cell_value(dest.x,dest.y,dest.z) = new_cost;
                            // And insert the selected cells in the map.
                            heap.insert(Heap::value_type(new_cost+heuristic(start, target),dest));
                        }
                    }
                }
                if (std::isnan(cell_value(target.x,target.y,target.z))) {
                    // No path found
                    ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,target.x,target.y);
                    std_msgs::Bool msg;
                    path_current_callback(msg);
                    return;
                }
                ROS_INFO("Planning completed");
                // Now extract the path by starting from goal and going through the
                // predecessors until the starting point
                std::list<cv::Point3i> lpath;
                while (target != start) {
                    assert(lpath.size()<1000000);
                    lpath.push_front(target);
                    cv::Vec3s p = predecessor(target.x,target.y,target.z);
                    target.x = p[0]; target.y = p[1]; target.z=p[2];
                }
                lpath.push_front(start);
                // Finally create a ROS path message
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = frame_id_;
                path.poses.resize(lpath.size());
                std::list<cv::Point3i>::const_iterator it = lpath.begin();
                unsigned int ipose = 0;
                while (it != lpath.end()) {
                    // time stamp is not updated because we're not creating a
                    // trajectory at this stage
                    path.poses[ipose].header = path.header;
                    cv::Point3i P = *it - og_center_;
                    path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                    path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                    path.poses[ipose].pose.orientation.x = 0;
                    path.poses[ipose].pose.orientation.y = 0;
                    path.poses[ipose].pose.orientation.z = P.z*M_PI/4;
                    path.poses[ipose].pose.orientation.w = 1;
                    ipose++;
                    it ++;
                }
                path_pub_.publish(path);
                ROS_INFO("Request completed");
            }
        }

        void path_current_callback(const std_msgs::Bool & msg){
            if(start_exploration){
                if (!ready) {
                    return;
                }
                ROS_INFO("Compute frontier");
                // this gets the current pose in transform
                tf::StampedTransform transform;
                listener_.waitForTransform(frame_id_,base_link_,ros::Time::now(),ros::Duration(1.0));
                listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
                cv::Point3i actual_pos;
                if (debug) {
                    actual_pos = og_center_;
                } else {
                    actual_pos = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)(round(tf::getYaw(transform.getRotation())/(M_PI/4)))%8)
                    + og_center_;
                }
                //Choose the best frontier point
                cv::Point nearest_P;
                cv::Point nearest_forw_P;
                double nearest = 1000000;
                double nearest_forward = 1000000;
                double distance;
                double orientation;

                std::vector<cv::Point> frontier = frontier_og;
                frontier.insert(frontier.end(),frontier_signal.begin(),frontier_signal.end());
                if(frontier.empty()){
                    ROS_INFO("Finish");
                    return;
                }
                for(cv::Point P : frontier){
                    distance=hypot(P.x-actual_pos.x,P.y-actual_pos.y);
                    if(distance<nearest && distance>(1/info_.resolution)) {
                        nearest=distance;
                        nearest_P=P;
                    }
                    if(distance<nearest_forward && distance>(1/info_.resolution)){
                        orientation = atan2(P.x-actual_pos.x,P.y-actual_pos.y);
                        orientation = std::fmod(std::fmod(orientation+M_PI/2 , 2*M_PI) + M_PI, 2*M_PI) - M_PI;
                        if(abs(tf::getYaw(transform.getRotation())-orientation)<=(45*M_PI/180)){
                            nearest_forward=distance;
                            nearest_forw_P=P;
                        }
                    }
                }
                ROS_INFO("nearest forward Point = (%d,%d)",nearest_forw_P.x,nearest_forw_P.y);
                ROS_INFO("size map pixel %d, %d", og_.rows,og_.cols);
                cv::Point3i target;
                if(nearest_forward == 1000000){
                    target.x=nearest_P.x;
                    target.y=nearest_P.y;
                }
                else{
                    target.x=nearest_forw_P.x;
                    target.y=nearest_forw_P.y;
                }
                int random = rand()%100;
                if(random>80){
                    random=rand()%frontier.size();
                    target.x=frontier[random].x;
                    target.y=frontier[random].y;
                }
                ROS_INFO("target position = (%d,%d)",target.x,target.y);
                orientation = atan2(target.x-actual_pos.x,target.y-actual_pos.y);
                orientation = std::fmod(std::fmod(orientation+M_PI/2 , 2*M_PI) + M_PI, 2*M_PI) - M_PI;
                target.z=(unsigned int)(round(orientation/(M_PI/4)))%8;
                geometry_msgs::PoseStamped target_msg;
                target_msg.header.stamp=ros::Time::now();
                target_msg.header.frame_id="map";
                target_msg.pose.position.x=target.x;
                target_msg.pose.position.y=target.y;
                target_msg.pose.position.z=target.z;
                target_msg.pose.orientation.x=0;
                target_msg.pose.orientation.y=0;
                target_msg.pose.orientation.z=0;
                target_msg.pose.orientation.w=0;
                ROS_INFO("target x = %d", target.x);
                target_pub_.publish(target_msg);
            }
        }

        void exploration_callback(const std_msgs::Bool & msg){
            start_exploration=msg.data;
        }

    public:
        OccupancyGridPlanner() : nh_("~") {
            int nbour = 4;
            ready = false;
            nh_.param("base_frame",base_link_,std::string("/body"));
            nh_.param("debug",debug,false);
            nh_.param("neighbourhood",nbour,nbour);
            nh_.param("radius",radius,0.15);
            switch (nbour) {
                case 4: neighbourhood_ = nbour; break;
                case 8: neighbourhood_ = nbour; break;
                default: 
                    ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)",nbour);
                    neighbourhood_ = nbour;
            }
            og_sub_ = nh_.subscribe("/map",1,&OccupancyGridPlanner::og_callback,this);
            og_signal_sub_ = nh_.subscribe("/wpa_cli/scan",1,&OccupancyGridPlanner::og_signal_callback,this);
            target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::path_pixel_callback,this);
            path_current = nh_.subscribe("/moving",1,&OccupancyGridPlanner::path_current_callback,this);
            exploration_sub = nh_.subscribe("/exploration",1,&OccupancyGridPlanner::exploration_callback,this);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
            target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1);
            create_signal_map=true;
            start_exploration=false;
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_planner");
    OccupancyGridPlanner ogp;
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}

