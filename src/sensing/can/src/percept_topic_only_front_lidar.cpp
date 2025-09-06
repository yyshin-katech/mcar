#include <ros/ros.h>
#include <perception_ros_msg/RsPerceptionMsg.h>
#include <perception_ros_msg/object_array_msg.h>
#include <perception_ros_msg/object_msg.h>
#include <std_msgs/String.h>
#include <map>
#include <string>
#include <sstream>

// 전역 변수로 객체 리스트를 유지합니다.
std::map<unsigned int, perception_ros_msg::object_msg> current_objects;

void callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& data) {
    static ros::Publisher pub = ros::NodeHandle().advertise<perception_ros_msg::object_array_msg>("/track_front_RS", 10);
    perception_ros_msg::object_array_msg transformed_msg;
    transformed_msg.time = ros::Time::now();

    // 입력 데이터의 객체를 업데이트합니다.
    for (const auto& obj : data->lidarframe.objects.objects) {
        const auto& coreinfo = obj.coreinfo;
        unsigned int obj_id = coreinfo.trakcer_id.data;

        perception_ros_msg::object_msg ads_obj;
        if (current_objects.find(obj_id) != current_objects.end()) {
            ads_obj = current_objects[obj_id];
        } else {
            current_objects[obj_id] = ads_obj;
        }

        ads_obj.id = obj_id;
        ads_obj.status = data->lidarframe.status.data;
        
        double time_predict_arr = ros::Time::now().toSec();  
        ads_obj.valid_level = std::to_string(static_cast<int>(time_predict_arr)) + ", 0";
        
        ads_obj.x = coreinfo.center.x.data;
        ads_obj.y = coreinfo.center.y.data;
        
        double direction_x = coreinfo.direction.x.data;
        double direction_y = coreinfo.direction.y.data;

        ads_obj.vx = coreinfo.velocity.x.data;
        ads_obj.vy = coreinfo.velocity.y.data;

        ads_obj.ax = coreinfo.acceleration.x.data;
        ads_obj.ay = coreinfo.acceleration.y.data;
        
        ads_obj.size_x = coreinfo.size.x.data;
        ads_obj.size_y = coreinfo.size.y.data;
        ads_obj.orientation = atan2(direction_y,direction_x);
        ads_obj.confidence = coreinfo.exist_confidence.data;

        // 🔹 필터링: x 값이 0 이상인 객체만 저장
        if (ads_obj.x >= 0) {
            transformed_msg.data.push_back(ads_obj);
        }
    }
    
    pub.publish(transformed_msg);

    std::ostringstream output;
    output << "---\ndata:\n";
    for (const auto& ads_obj : transformed_msg.data) {
        output << "  - \n";
        output << "    id: " << ads_obj.id << "\n";
        output << "    status: " << ads_obj.status << "\n";
        output << "    valid_level: \"" << ads_obj.valid_level << "\"\n";
        output << "    x: " << ads_obj.x << "\n";
        output << "    y: " << ads_obj.y << "\n";
        output << "    vx: " << ads_obj.vx << "\n";
        output << "    vy: " << ads_obj.vy << "\n";
        output << "    ax: " << ads_obj.ax << "\n";
        output << "    ay: " << ads_obj.ay << "\n";
        output << "    size_x: " << ads_obj.size_x << "\n";
        output << "    size_y: " << ads_obj.size_y << "\n";
        output << "    orientation: " << ads_obj.orientation << "\n";
        output << "    confidence: " << ads_obj.confidence << "\n";
    }
    
    output << "time:\n  secs: " << transformed_msg.time.sec << "\n  nsecs: " << transformed_msg.time.nsec << "\n";
    ROS_INFO_STREAM(output.str());
}

void transformer_node(int argc, char** argv) {
    ros::init(argc, argv, "percept_topic_only_front_lidar");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/percept_topic", 10, callback); // 라이다 모듈 publisher 토픽 명 
    ros::spin();
}

int main(int argc, char** argv) {
    transformer_node(argc, argv);
    return 0;
}





