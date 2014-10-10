#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <fstream>
#include <string>

class WaypointsNavigation{
public:
    WaypointsNavigation() :
        has_activate_(false),
        move_base_action_("move_base", true)
    {
        while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
        {
            ROS_INFO("Waiting...");
        }
        
        
        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
        private_nh.param("world_frame", world_frame_, std::string("/map"));
        std::string filename = "";
        private_nh.param<std::string>("filename", filename, filename);
        if(filename != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            readFile(filename);
        }
        for(int i=0; i < waypoints_.size(); i++){
            ROS_INFO_STREAM("waypoints \n" << waypoints_[i]);
        }
    }

    bool readFile(const std::string &filename){
        waypoints_.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
                return false;
            }

            YAML::Node node;
            node = YAML::Load(ifs);
            const YAML::Node &wp_node_tmp = node["waypoints"];
            const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){
                    geometry_msgs::PointStamped point;

                    point.point.x = (*wp_node)[i]["point"]["x"].as<double>();
                    point.point.y = (*wp_node)[i]["point"]["y"].as<double>();
                    point.point.z = (*wp_node)[i]["point"]["z"].as<double>();

                    waypoints_.push_back(point);

                }
            }else{
                return false;
            }

            const YAML::Node &fp_node_tmp = node["finish_pose"];
            const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;

            if(fp_node != NULL){
                finish_pose_.pose.position.x = (*fp_node)["pose"]["position"]["x"].as<double>();
                finish_pose_.pose.position.y = (*fp_node)["pose"]["position"]["y"].as<double>();
                finish_pose_.pose.position.z = (*fp_node)["pose"]["position"]["z"].as<double>();

                finish_pose_.pose.orientation.x = (*fp_node)["pose"]["orientation"]["x"].as<double>();
                finish_pose_.pose.orientation.y = (*fp_node)["pose"]["orientation"]["y"].as<double>();
                finish_pose_.pose.orientation.z = (*fp_node)["pose"]["orientation"]["z"].as<double>();
                finish_pose_.pose.orientation.w = (*fp_node)["pose"]["orientation"]["w"].as<double>();
            }else{
                return false;
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        has_activate_ = true;
        return true;
    }

    bool shouldSendGoal(){
        bool ret = true;
        actionlib::SimpleClientGoalState state = move_base_action_.getState();
        if((state != actionlib::SimpleClientGoalState::ACTIVE) &&
           (state != actionlib::SimpleClientGoalState::PENDING) && 
           (state != actionlib::SimpleClientGoalState::RECALLED) &&
           (state != actionlib::SimpleClientGoalState::PREEMPTED))
        {
            ret = false;
        }

        if(waypoints_.empty()){
            ret = false;
        }

        return ret;
    }

    bool navigationFinished(const geometry_msgs::Point &dest, double dist_err = 0.5){
        tf::StampedTransform robot_gl;
        try{
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        }catch(tf::TransformException &e){
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }
        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

        return dist < dist_err;
    }

    void startNavigationGL(const geometry_msgs::Point &dest){
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position = dest;
        move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        
        move_base_action_.sendGoal(move_base_goal);
    }

    void run(){
        ros::Rate rate(10.0);
        while(ros::ok()){
            for(int i=0; i < waypoints_.size(); i++){
                if(!ros::ok()) break;
                
                startNavigationGL(waypoints_[i].point);
                while(!navigationFinished(waypoints_[i].point) && ros::ok()){
                    ros::spinOnce();
                    rate.sleep();
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    std::vector<geometry_msgs::PointStamped> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    bool has_activate_;
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();

    return 0;
}
