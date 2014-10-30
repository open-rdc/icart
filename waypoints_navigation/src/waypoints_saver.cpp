#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
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
        waypoints_viz_sub_ = nh_.subscribe("waypoints_viz", 1, &WaypointsSaver::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh_.subscribe("waypoints_joy", 1, &WaypointsSaver::waypointsJoyCallback, this);
        finish_pose_sub_ = nh_.subscribe("finish_pose", 1, &WaypointsSaver::finishPoseCallback, this);

        ros::NodeHandle private_nh("~");
        private_nh.param("filename", filename_, filename_);
        private_nh.param("save_joy_button", save_joy_button_, 0);
        private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
        private_nh.param("world_frame", world_frame_, std::string("/map"));
    }

    void waypointsJoyCallback(const sensor_msgs::Joy &msg){
        static ros::Time saved_time(0.0);
        //ROS_INFO_STREAM("joy = " << msg);
        if(msg.buttons[save_joy_button_] == 1 && (ros::Time::now() - saved_time).toSec() > 3.0){
            tf::StampedTransform robot_gl;
            try{
                tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
                geometry_msgs::PointStamped point;
                point.point.x = robot_gl.getOrigin().x();
                point.point.y = robot_gl.getOrigin().y();
                point.point.z = robot_gl.getOrigin().z();
                waypoints_.push_back(point);
                saved_time = ros::Time::now();
            }catch(tf::TransformException &e){
                ROS_WARN_STREAM("tf::TransformException: " << e.what());
            }
        }
    }
    
    void waypointsVizCallback(const geometry_msgs::PointStamped &msg){
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
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::Subscriber finish_pose_sub_;
    std::vector<geometry_msgs::PointStamped> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    ros::NodeHandle nh_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "waypoints_saver");
    WaypointsSaver saver;
    saver.run();

    return 0;
}
