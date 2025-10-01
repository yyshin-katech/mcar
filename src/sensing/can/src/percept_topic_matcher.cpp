#include <ros/ros.h>
#include <perception_ros_msg/RsPerceptionMsg.h>
#include <perception_ros_msg/object_array_msg.h>
#include <perception_ros_msg/object_msg.h>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <set>

// ---- 객체 상태(옵션) ----
struct ObjectState {
    perception_ros_msg::object_msg obj;
    ros::Time last_time;
    double prev_x;
    double prev_y;
};

// ---- 전역 매핑/상태 ----
std::map<unsigned int, uint8_t> object_to_can_id;   // 원본 tracker_id -> 할당된 CAN ID(1~254)
std::map<uint8_t, unsigned int> can_id_to_object;   // CAN ID -> 원본 tracker_id
std::set<uint8_t> used_can_ids;                     // 현재 사용 중인 CAN ID
std::map<unsigned int, ObjectState> current_objects;// 원본 tracker_id -> 상태
std::map<unsigned int, int> absence_count;          // 원본 tracker_id -> 부재 프레임 수
const int max_absence_threshold = 20;               // 20프레임 미등장 시 회수
const int max_objects_to_publish = 14;              // 최대 전송 객체 수
const double min_confidence_threshold = 0.90;        // 최소 confidence 임계값

// 빈 CAN ID 할당(1~254). 이미 있으면 그대로 반환하고 부재 카운트 0으로 초기화
uint8_t assignCanID(unsigned int object_id) {
    auto it = object_to_can_id.find(object_id);
    if (it != object_to_can_id.end()) {
        absence_count[object_id] = 0;
        return it->second;
    }
    for (uint8_t i = 1; i < 255; ++i) {
        if (used_can_ids.count(i) == 0) {
            object_to_can_id[object_id] = i;
            can_id_to_object[i] = object_id;
            used_can_ids.insert(i);
            absence_count[object_id] = 0;
            return i;
        }
    }
    return 0; // 가용 ID 없음(이 경우 0으로 표기)
}

// 부재 객체 회수: 모든 absence_count 1 증가 후, 20 이상이면 CAN ID 반환 및 상태 제거
void cleanupStaleIDs() {
    std::vector<unsigned int> expired_ids;
    for (auto &entry : absence_count) {
        entry.second += 1;
        if (entry.second >= max_absence_threshold) {
            expired_ids.push_back(entry.first);
        }
    }
    for (unsigned int obj_id : expired_ids) {
        auto it_obj_can = object_to_can_id.find(obj_id);
        if (it_obj_can != object_to_can_id.end()) {
            uint8_t can_id = it_obj_can->second;
            used_can_ids.erase(can_id);
            can_id_to_object.erase(can_id);
            object_to_can_id.erase(obj_id);
        }
        current_objects.erase(obj_id);
        absence_count.erase(obj_id);
    }
}

void callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& data) {
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<perception_ros_msg::object_array_msg>("/track_Multi_RS", 10);

    perception_ros_msg::object_array_msg transformed_msg;
    transformed_msg.time = ros::Time::now();

    // 1) 부재 객체 회수 시도
    cleanupStaleIDs();

    std::set<unsigned int> current_frame_ids;

    // priority 정렬을 위해 priority_id와 attention_type, confidence를 함께 보관
    struct PriorityObj {
        unsigned int priority_id;
        int attention_type;
        double confidence;
        perception_ros_msg::object_msg obj;
    };
    
    std::vector<PriorityObj> all_objects;
    std::vector<PriorityObj> attention1_objects;  // attention_type이 1인 객체들 (confidence > 0.5)
    std::vector<PriorityObj> attention0_objects;  // attention_type이 0인 객체들 (confidence > 0.5)

    // 현재 프레임에서 수신된 전체 객체 수
    const size_t total_tracks = data->lidarframe.objects.objects.size();
    int total_filtered_out = 0;  // confidence <= 0.5로 필터링된 객체 수

    // 2) 현재 프레임 처리 - 모든 객체를 처리 (attention_type 구분 없이)
    for (const auto& obj : data->lidarframe.objects.objects) {
        const auto& coreinfo = obj.coreinfo;

        unsigned int tracker_id  = coreinfo.trakcer_id.data;   // 원본 트래커 ID (필드명 유지)
        unsigned int priority_id = coreinfo.priority_id.data;  // 우선순위 정렬용
        int attention_type = coreinfo.attention_type.data;     // attention_type 추출
        double confidence = coreinfo.exist_confidence.data;    // confidence 추출
        
        // confidence 필터링 - 0.5 이하인 객체는 건너뛰기
        if (confidence <= min_confidence_threshold) {
            total_filtered_out++;
            continue;
        }
        
        current_frame_ids.insert(tracker_id);

        // CAN ID 할당/획득
        uint8_t tracker_can_id = assignCanID(tracker_id);

        // 좌표/속도 등 추출
        double curr_x = coreinfo.center.x.data;
        double curr_y = coreinfo.center.y.data;
        double vx = coreinfo.velocity.x.data;
        double vy = coreinfo.velocity.y.data;
        double ax = coreinfo.acceleration.x.data;
        double ay = coreinfo.acceleration.y.data;
        double direction_x = coreinfo.direction.x.data;
        double direction_y = coreinfo.direction.y.data;       

        ros::Time curr_time = ros::Time::now();

        // 메시지 채우기
        perception_ros_msg::object_msg ads_obj;
        ads_obj.id = tracker_can_id;                               // ★ 할당된 CAN ID
        ads_obj.status = coreinfo.type.data;
        
        // ★ valid_level: "원본 tracker_id, total_tracks, priority_id"
        ads_obj.valid_level = std::to_string(tracker_id) + ", " +
                              std::to_string(static_cast<unsigned int>(total_tracks)) + ", " +
                              std::to_string(priority_id);

        ads_obj.x = curr_x;
        ads_obj.y = curr_y;
        ads_obj.vx = vx;
        ads_obj.vy = vy;
        ads_obj.ax = ax;
        ads_obj.ay = ay;

        ads_obj.size_x = coreinfo.size.x.data;
        ads_obj.size_y = coreinfo.size.y.data;

        ads_obj.orientation = atan2(direction_y, direction_x);
        ads_obj.orientation_v = ads_obj.orientation;
        ads_obj.confidence = confidence;

        // 기존 코드에 있던 최근접점 정보도 그대로 매핑
        ads_obj.nearest_point_x = coreinfo.nearest_point.x.data;
        ads_obj.nearest_point_y = coreinfo.nearest_point.y.data;
        ads_obj.nearest_point_z = coreinfo.nearest_point.z.data;

        // 상태 저장(옵션)
        current_objects[tracker_id] = {ads_obj, curr_time, curr_x, curr_y};

        // PriorityObj 구조체 생성
        PriorityObj p_obj = {priority_id, attention_type, confidence, ads_obj};
        all_objects.push_back(p_obj);

        // attention_type별로 분류 (confidence > 0.5인 객체만 이미 필터링됨)
        if (attention_type == 1) {
            attention1_objects.push_back(p_obj);
        } else if (attention_type == 0) {
            attention0_objects.push_back(p_obj);
        }
    }

    // 3) 이번 프레임에서 관측된 객체는 부재 카운트 0으로 리셋
    for (unsigned int id : current_frame_ids) {
        absence_count[id] = 0;
    }

    // 4) attention_type이 1인 객체들을 priority_id 오름차순으로 정렬
    std::sort(attention1_objects.begin(), attention1_objects.end(),
              [](const PriorityObj& a, const PriorityObj& b) {
                  return a.priority_id < b.priority_id;
              });

    // attention_type이 0인 객체들도 priority_id 오름차순으로 정렬
    std::sort(attention0_objects.begin(), attention0_objects.end(),
              [](const PriorityObj& a, const PriorityObj& b) {
                  return a.priority_id < b.priority_id;
              });

    // 5) 최대 14개까지 선택
    std::vector<PriorityObj> objects_to_publish;
    
    // 먼저 attention_type이 1인 객체들을 추가 (confidence > 0.5)
    for (const auto& p_obj : attention1_objects) {
        if (objects_to_publish.size() >= max_objects_to_publish) {
            break;
        }
        objects_to_publish.push_back(p_obj);
    }
    
    // 14개가 안 채워지면 attention_type이 0인 객체들로 채우기 (confidence > 0.5)
    for (const auto& p_obj : attention0_objects) {
        if (objects_to_publish.size() >= max_objects_to_publish) {
            break;
        }
        objects_to_publish.push_back(p_obj);
    }

    // 6) 선택된 객체들만 publish
    for (const auto& p_obj : objects_to_publish) {
        transformed_msg.data.push_back(p_obj.obj);
    }
    pub.publish(transformed_msg);

    // 7) 터미널 출력 (실제 publish된 객체들만 출력)
    std::ostringstream output;
    output << "---\ndata (confidence > " << min_confidence_threshold 
           << ", attention_type=1 prioritized, sorted by priority_id, limited to " << max_objects_to_publish << "):\n";
    output << "Total received objects: " << total_tracks
           << ", Filtered out (confidence <= " << min_confidence_threshold << "): " << total_filtered_out
           << ", Passed filter: " << all_objects.size() << "\n";
    output << "After filtering - Attention=1: " << attention1_objects.size()
           << ", Attention=0: " << attention0_objects.size() 
           << ", Published: " << objects_to_publish.size() << "\n";
    
    // attention_type별 게시된 객체 수 계산
    int published_attention1 = 0;
    int published_attention0 = 0;
    for (const auto& p_obj : objects_to_publish) {
        if (p_obj.attention_type == 1) {
            published_attention1++;
        } else if (p_obj.attention_type == 0) {
            published_attention0++;
        }
    }
    output << "Published breakdown - Attention=1: " << published_attention1 
           << ", Attention=0: " << published_attention0 << "\n";
    
    // confidence 통계 (published objects)
    if (!objects_to_publish.empty()) {
        double min_conf = 1.0, max_conf = 0.0, avg_conf = 0.0;
        for (const auto& p_obj : objects_to_publish) {
            min_conf = std::min(min_conf, p_obj.confidence);
            max_conf = std::max(max_conf, p_obj.confidence);
            avg_conf += p_obj.confidence;
        }
        avg_conf /= objects_to_publish.size();
        output << "Published confidence stats - Min: " << min_conf 
               << ", Max: " << max_conf 
               << ", Avg: " << avg_conf << "\n";
    }
    
    for (const auto& p_obj : objects_to_publish) {
        const auto& ads_obj = p_obj.obj;
        output << "  - \n";
        output << "    priority_id: " << p_obj.priority_id << "\n";
        output << "    attention_type: " << p_obj.attention_type << "\n";
        output << "    id (assigned CAN ID): " << static_cast<unsigned int>(ads_obj.id) << "\n";
        output << "    status: " << static_cast<int>(ads_obj.status) << "\n";
        output << "    valid_level (tracker_id, total_tracks, priority): \"" << ads_obj.valid_level << "\"\n";
        output << "    confidence: " << ads_obj.confidence << "\n";
        output << "    x: " << ads_obj.x << "\n";
        output << "    y: " << ads_obj.y << "\n";
        output << "    vx: " << ads_obj.vx << "\n";
        output << "    vy: " << ads_obj.vy << "\n";
        output << "    ax: " << ads_obj.ax << "\n";
        output << "    ay: " << ads_obj.ay << "\n";
        output << "    size_x: " << ads_obj.size_x << "\n";
        output << "    size_y: " << ads_obj.size_y << "\n";
        output << "    orientation: " << ads_obj.orientation << "\n";
        output << "    orientation_v: " << ads_obj.orientation_v << "\n";
        output << "    nearest_point_x: " << ads_obj.nearest_point_x << "\n";
        output << "    nearest_point_y: " << ads_obj.nearest_point_y << "\n";
        output << "    nearest_point_z: " << ads_obj.nearest_point_z << "\n";
    }
    output << "time:\n  secs: " << transformed_msg.time.sec
           << "\n  nsecs: " << transformed_msg.time.nsec << "\n";
    ROS_INFO_STREAM(output.str());
}

void transformer_node(int argc, char** argv) {
    ros::init(argc, argv, "percept_topic_matcher");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/percept_topic", 10, callback);
    ros::spin();
}

int main(int argc, char** argv) {
    transformer_node(argc, argv);
    return 0;
}




// #include <ros/ros.h>
// #include <perception_ros_msg/RsPerceptionMsg.h>
// #include <perception_ros_msg/object_array_msg.h>
// #include <perception_ros_msg/object_msg.h>
// #include <map>
// #include <vector>
// #include <string>
// #include <sstream>
// #include <cmath>
// #include <algorithm>
// #include <set>

// // ---- 객체 상태(옵션) ----
// struct ObjectState {
//     perception_ros_msg::object_msg obj;
//     ros::Time last_time;
//     double prev_x;
//     double prev_y;
// };

// // ---- 전역 매핑/상태 ----
// std::map<unsigned int, uint8_t> object_to_can_id;   // 원본 tracker_id -> 할당된 CAN ID(1~254)
// std::map<uint8_t, unsigned int> can_id_to_object;   // CAN ID -> 원본 tracker_id
// std::set<uint8_t> used_can_ids;                     // 현재 사용 중인 CAN ID
// std::map<unsigned int, ObjectState> current_objects;// 원본 tracker_id -> 상태
// std::map<unsigned int, int> absence_count;          // 원본 tracker_id -> 부재 프레임 수
// const int max_absence_threshold = 20;               // 20프레임 미등장 시 회수

// // 빈 CAN ID 할당(1~254). 이미 있으면 그대로 반환하고 부재 카운트 0으로 초기화
// uint8_t assignCanID(unsigned int object_id) {
//     auto it = object_to_can_id.find(object_id);
//     if (it != object_to_can_id.end()) {
//         absence_count[object_id] = 0;
//         return it->second;
//     }
//     for (uint8_t i = 1; i < 255; ++i) {
//         if (used_can_ids.count(i) == 0) {
//             object_to_can_id[object_id] = i;
//             can_id_to_object[i] = object_id;
//             used_can_ids.insert(i);
//             absence_count[object_id] = 0;
//             return i;
//         }
//     }
//     return 0; // 가용 ID 없음(이 경우 0으로 표기)
// }

// // 부재 객체 회수: 모든 absence_count 1 증가 후, 20 이상이면 CAN ID 반환 및 상태 제거
// void cleanupStaleIDs() {
//     std::vector<unsigned int> expired_ids;
//     for (auto &entry : absence_count) {
//         entry.second += 1;
//         if (entry.second >= max_absence_threshold) {
//             expired_ids.push_back(entry.first);
//         }
//     }
//     for (unsigned int obj_id : expired_ids) {
//         auto it_obj_can = object_to_can_id.find(obj_id);
//         if (it_obj_can != object_to_can_id.end()) {
//             uint8_t can_id = it_obj_can->second;
//             used_can_ids.erase(can_id);
//             can_id_to_object.erase(can_id);
//             object_to_can_id.erase(obj_id);
//         }
//         current_objects.erase(obj_id);
//         absence_count.erase(obj_id);
//     }
// }

// void callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& data) {
//     static ros::NodeHandle nh;
//     static ros::Publisher pub = nh.advertise<perception_ros_msg::object_array_msg>("/track_Multi_RS", 10);

//     perception_ros_msg::object_array_msg transformed_msg;
//     transformed_msg.time = ros::Time::now();

//     // 1) 부재 객체 회수 시도
//     cleanupStaleIDs();

//     std::set<unsigned int> current_frame_ids;

//     // priority 정렬을 위해 priority_id를 함께 보관
//     struct PriorityObj {
//         unsigned int priority_id;
//         perception_ros_msg::object_msg obj;
//     };
//     std::vector<PriorityObj> collected_objects;

//     // 2) 현재 프레임 처리
//     for (const auto& obj : data->lidarframe.objects.objects) {
//         const auto& coreinfo = obj.coreinfo;

//         // attention_type == 1 인 객체만 처리 (원 코드와 동일)
//         if (coreinfo.attention_type.data != 1) continue;

//         unsigned int tracker_id  = coreinfo.trakcer_id.data;   // 원본 트래커 ID (필드명 유지)
//         unsigned int priority_id = coreinfo.priority_id.data;  // 우선순위 정렬용
//         current_frame_ids.insert(tracker_id);

//         // CAN ID 할당/획득
//         uint8_t tracker_can_id = assignCanID(tracker_id);

//         // 좌표/속도 등 추출
//         double curr_x = coreinfo.center.x.data;
//         double curr_y = coreinfo.center.y.data;
//         double vx = coreinfo.velocity.x.data;
//         double vy = coreinfo.velocity.y.data;
//         double direction_x = coreinfo.direction.x.data;
//         double direction_y = coreinfo.direction.y.data;       

//         ros::Time curr_time = ros::Time::now();

//         // 메시지 채우기
//         perception_ros_msg::object_msg ads_obj;
//         ads_obj.id = tracker_can_id;                               // ★ 할당된 CAN ID
//         // ads_obj.status = data->lidarframe.status.data;
//         ads_obj.status = coreinfo.type.data;
//         // ★ valid_level: "원본 tracker_id, priority_id"
//         ads_obj.valid_level = std::to_string(tracker_id) + ", " +
//                               std::to_string(priority_id);

//         ads_obj.x = curr_x;
//         ads_obj.y = curr_y;
//         ads_obj.vx = vx;
//         ads_obj.vy = vy;
//         ads_obj.ax = coreinfo.acceleration.x.data;
//         ads_obj.ay = coreinfo.acceleration.y.data;

//         ads_obj.size_x = coreinfo.size.x.data;
//         ads_obj.size_y = coreinfo.size.y.data;

//         ads_obj.orientation = atan2(direction_y, direction_x);
//         ads_obj.orientation_v = ads_obj.orientation;
//         ads_obj.confidence = coreinfo.exist_confidence.data;

//         // 기존 코드에 있던 최근접점 정보도 그대로 매핑
//         ads_obj.nearest_point_x = coreinfo.nearest_point.x.data;
//         ads_obj.nearest_point_y = coreinfo.nearest_point.y.data;
//         ads_obj.nearest_point_z = coreinfo.nearest_point.z.data;

//         // 상태 저장(옵션)
//         current_objects[tracker_id] = {ads_obj, curr_time, curr_x, curr_y};

//         // 정렬용 컨테이너에 push
//         collected_objects.push_back({priority_id, ads_obj});
//     }

//     // 3) 이번 프레임에서 관측된 객체는 부재 카운트 0으로 리셋
//     for (unsigned int id : current_frame_ids) {
//         absence_count[id] = 0;
//     }

//     // 4) priority_id 오름차순 정렬
//     std::sort(collected_objects.begin(), collected_objects.end(),
//               [](const PriorityObj& a, const PriorityObj& b) {
//                   return a.priority_id < b.priority_id;
//               });

//     // 5) 상위 14개만 publish (퍼블리시 순서 == priority_id 오름차순)
//     const size_t max_out = std::min(collected_objects.size(), static_cast<size_t>(14));
//     for (size_t i = 0; i < max_out; ++i) {
//         transformed_msg.data.push_back(collected_objects[i].obj);
//     }
//     pub.publish(transformed_msg);

//     // 6) 터미널 출력도 publish된 순서와 동일(= priority_id 오름차순, 최대 14개)
//     std::ostringstream output;
//     output << "---\ndata (sorted by priority_id, published subset):\n";
//     for (size_t i = 0; i < max_out; ++i) {
//         const auto& p_obj  = collected_objects[i];
//         const auto& ads_obj = p_obj.obj;
//         output << "  - \n";
//         output << "    priority_id: " << p_obj.priority_id << "\n";
//         output << "    id (assigned CAN ID): " << static_cast<unsigned int>(ads_obj.id) << "\n";
//         output << "    status: " << static_cast<int>(ads_obj.status) << "\n";
//         // ★ 문구도 업데이트
//         output << "    valid_level (original_id, priority_id): \"" << ads_obj.valid_level << "\"\n";
//         output << "    x: " << ads_obj.x << "\n";
//         output << "    y: " << ads_obj.y << "\n";
//         output << "    vx: " << ads_obj.vx << "\n";
//         output << "    vy: " << ads_obj.vy << "\n";
//         output << "    ax: " << ads_obj.ax << "\n";
//         output << "    ay: " << ads_obj.ay << "\n";
//         output << "    size_x: " << ads_obj.size_x << "\n";
//         output << "    size_y: " << ads_obj.size_y << "\n";
//         output << "    orientation: " << ads_obj.orientation << "\n";
//         output << "    orientation_v: " << ads_obj.orientation_v << "\n";
//         output << "    confidence: " << ads_obj.confidence << "\n";
//         output << "    nearest_point_x: " << ads_obj.nearest_point_x << "\n";
//         output << "    nearest_point_y: " << ads_obj.nearest_point_y << "\n";
//         output << "    nearest_point_z: " << ads_obj.nearest_point_z << "\n";
//     }
//     output << "time:\n  secs: " << transformed_msg.time.sec
//            << "\n  nsecs: " << transformed_msg.time.nsec << "\n";
//     ROS_INFO_STREAM(output.str());
// }

// void transformer_node(int argc, char** argv) {
//     ros::init(argc, argv, "percept_topic_matcher");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe("/percept_topic", 10, callback);
//     ros::spin();
// }

// int main(int argc, char** argv) {
//     transformer_node(argc, argv);
//     return 0;
// }
