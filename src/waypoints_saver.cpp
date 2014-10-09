#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <vector>
#include <fstream>
#include <string>

class WaypointsSaver{
public:
    WaypointsSaver() : 
        filename_("waypoints.yaml")
    {
        waypoints_sub_ = nh_.subscribe("waypoints", 1, &WaypointsSaver::waypointsCallback, this);
        finish_pose_sub_ = nh_.subscribe("finish_pose", 1, &WaypointsSaver::finishPoseCallback, this);

        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("filename", filename_, filename_);
    }
    
    void waypointsCallback(const geometry_msgs::PointStamped &msg){
        ROS_INFO_STREAM("point = " << msg);
        waypoints_.push_back(msg);
    }

    void finishPoseCallback(const geometry_msgs::PoseStamped &msg){
        finish_pose_ = msg;
        save();
        waypoints_.clear();
    }
    
    void save(){
        std::ofstream ofs(filename_.c_str(), std::ios::out);
        
        ofs << "waypoints:" << std::endl;
        for(int i=0; i < waypoints_.size(); i++){
            ofs << "    " << "- point:" << std::endl;
            ofs << "        x: " << waypoints_[i].point.x << std::endl;
            ofs << "        y: " << waypoints_[i].point.y << std::endl;
            ofs << "        z: " << waypoints_[i].point.z << std::endl;
        }
        
        ofs << "finish_pose:"           << std::endl;
        ofs << "    header:"            << std::endl;
        ofs << "        seq: "          << finish_pose_.header.seq << std::endl;
        ofs << "        stamp: "        << finish_pose_.header.stamp << std::endl;
        ofs << "        frame_id: "     << finish_pose_.header.frame_id << std::endl;;
        ofs << "    pose:"              << std::endl;
        ofs << "        position:"      << std::endl;
        ofs << "            x: "        << finish_pose_.pose.position.x << std::endl;
        ofs << "            y: "        << finish_pose_.pose.position.y << std::endl;
        ofs << "            z: "        << finish_pose_.pose.position.z << std::endl;
        ofs << "        orientation:"   << std::endl;
        ofs << "            x: "        << finish_pose_.pose.orientation.x << std::endl;
        ofs << "            y: "        << finish_pose_.pose.orientation.y << std::endl;
        ofs << "            z: "        << finish_pose_.pose.orientation.z << std::endl;
        ofs << "            w: "        << finish_pose_.pose.orientation.w << std::endl;

        ofs.close();

        ROS_INFO_STREAM("write success");
    }
    
    void run(){
        ros::spin();
    }
    
private:
    ros::Subscriber waypoints_sub_;
    ros::Subscriber finish_pose_sub_;
    std::vector<geometry_msgs::PointStamped> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    ros::NodeHandle nh_;
    std::string filename_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "waypoints_saver");
    WaypointsSaver saver;
    saver.run();

    return 0;
}
