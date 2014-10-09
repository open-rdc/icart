#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <vector>
#include <fstream>
#include <string>

class WaypointsSaver{
public:
    WaypointsSaver(){
        waypoints_sub_ = nh_.subscribe("waypoints", 1, &WaypointsSaver::waypointsCallback, this);
        syscommand_sub_ = nh_.subscribe("syscommand", 1, &WaypointsSaver::syscommandCallback, this);
    }
    
    void waypointsCallback(const geometry_msgs::PointStamped &msg){
        ROS_INFO_STREAM("point = " << msg);
        waypoints_.push_back(msg);
    }

    void syscommandCallback(const std_msgs::String &msg){
        if(msg.data == "save"){
            save();
        }else if(msg.data == "clear"){
            waypoints_.clear();
        }
    }

    void save(){
        std::ofstream ofs("/home/daikimaekawa/hoge.yaml", std::ios::out);
        
        ofs << "waypoints:" << std::endl;
        for(int i=0; i < waypoints_.size(); i++){
            ofs << "    " << "- point:" << std::endl;
            ofs << "        x: " << waypoints_[i].point.x << std::endl;
            ofs << "        y: " << waypoints_[i].point.y << std::endl;
            ofs << "        z: " << waypoints_[i].point.z << std::endl;
        }
        ofs.close();

        ROS_INFO_STREAM("write success");
    }
    
    void run(){
        ros::spin();
    }
    
private:
    ros::Subscriber waypoints_sub_;
    ros::Subscriber syscommand_sub_;
    std::vector<geometry_msgs::PointStamped> waypoints_;
    std::string syscommand_;
    ros::NodeHandle nh_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "waypoints_saver");
    WaypointsSaver saver;
    saver.run();

    return 0;
}
