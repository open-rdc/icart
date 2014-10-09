#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PointStamped.h>

#include <vector>
#include <fstream>
#include <string>

class WaypointsNavigation{
public:
    WaypointsNavigation(){
        ros::NodeHandle private_nh("~");
        std::string filename = "";
        private_nh.param<std::string>("filename", filename, filename);
        if(filename != ""){
            ROS_INFO_STREAM("Read waypoints from " << filename);
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
            }

            const YAML::Node &fp_node_tmp = node["finish_pose"];
            const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;

            if(fp_node != NULL){
                //TODO
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        return true;
    }

private:
    std::vector<geometry_msgs::PointStamped> waypoints_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;

    return 0;
}
