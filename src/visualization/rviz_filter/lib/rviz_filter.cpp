#include <ros/ros.h>
// #include <perception_ros_msg/RsPerceptionMsg.h>
#include <rviz_filter.h>
#include <iomanip> //text message
#include <cmath> // use math

RVIZ_FILTER::RVIZ_FILTER()
{

}

RVIZ_FILTER::~RVIZ_FILTER()
{

}

void RVIZ_FILTER::end(int sig)
{
    if(sig == 2)
    {
        std::cout << "shutdown rviz filter" << std::endl;
    }
}

void RVIZ_FILTER::getOrientationFromDirection(visualization_msgs::Marker &marker, const double &orientation)
{
    visualization_msgs::Marker ros_marker;
    // double yaw = std::atan2(direction.y.data, direction.x.data);
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, orientation);
    tf::quaternionTFToMsg(quat, ros_marker.pose.orientation);
    marker.pose.orientation.x = ros_marker.pose.orientation.x;
    marker.pose.orientation.y = ros_marker.pose.orientation.y;
    marker.pose.orientation.z = ros_marker.pose.orientation.z;
    marker.pose.orientation.w = ros_marker.pose.orientation.w;
    
}

void RVIZ_FILTER::perception_info_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

}

void RVIZ_FILTER::percept_callback(const perception_ros_msg::object_array_msg::ConstPtr &msg)
{
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        const auto &obj = msg->data[i];
        
        // Target Shape =================================
        visualization_msgs::Marker cube_marker;

        // Orientation
        getOrientationFromDirection(cube_marker, obj.orientation);

        cube_marker.header.frame_id = "ego_frame"; // frame ID (DEFAULT= ego_frame)
        cube_marker.header.stamp = now;
        cube_marker.ns = "detected_objects";
        cube_marker.id = i; // 각 마커마다 고유 ID 필요
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;

        // 위치
        cube_marker.pose.position.x = obj.x;
        cube_marker.pose.position.y = obj.y;
        cube_marker.pose.position.z = 0.0;

        // 크기 (size)
        cube_marker.scale.x = obj.size_x;
        cube_marker.scale.y = obj.size_y;
        cube_marker.scale.z = 1.0;
        
        // 색상 (랜덤 또는 고정)
        cube_marker.color.r = 1.0;
        cube_marker.color.g = 0.0;
        cube_marker.color.b = 0.0;
        cube_marker.color.a = 0.8;

        cube_marker.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(cube_marker);

        //================== Target Direction (Orientation)==========
        
        double orientation = obj.orientation;

        visualization_msgs::Marker tpose_marker;
        tpose_marker.header.frame_id = "ego_frame";
        tpose_marker.header.stamp = now;
        tpose_marker.ns = "target_pose_arrow";
        tpose_marker.id = i;
        tpose_marker.type = visualization_msgs::Marker::ARROW;
        tpose_marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point base, goal;
        base.x = obj.x;
        base.y = obj.y;
        base.z = 1;

        double arrow = 3.0;
        goal.x = obj.x + arrow * std::cos(orientation);
        goal.y = obj.y + arrow * std::sin(orientation);
        goal.z = 1;

        tpose_marker.points.push_back(base);
        tpose_marker.points.push_back(goal);
        
        tpose_marker.scale.x = 0.3;
        tpose_marker.scale.y = 0.5;
        tpose_marker.scale.z = 0.5;

        tpose_marker.color.r = 0.0;
        tpose_marker.color.g = 0.0;
        tpose_marker.color.b = 1.0;
        tpose_marker.color.a = 1.0;

        tpose_marker.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(tpose_marker);

        // --- ARROW marker (vx, vy base Heading Direction) ---
        if (std::abs(obj.vx) > 1e-3 || std::abs(obj.vy) > 1e-3)  // 정지 상태는 제외 (0 ~ 0.01) == stop
        {
            double heading = std::atan2(obj.vy, obj.vx);

            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = "ego_frame";
            arrow_marker.header.stamp = now;
            arrow_marker.ns = "direction_arrow";
            arrow_marker.id = i;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start, end;
            start.x = obj.x;
            start.y = obj.y;
            start.z = 0.5;

            double arrow_len = 3.0;
            end.x = obj.x + arrow_len * std::cos(heading);
            end.y = obj.y + arrow_len * std::sin(heading);
            end.z = 0.5;

            arrow_marker.points.push_back(start);
            arrow_marker.points.push_back(end);

            arrow_marker.scale.x = 0.3;
            arrow_marker.scale.y = 0.5;
            arrow_marker.scale.z = 0.5;

            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 1.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0;

            arrow_marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(arrow_marker);
    }

    // --- TEXT marker (ID + orientation) ---
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "ego_frame";
    text_marker.header.stamp = now;
    text_marker.ns = "id_orientation_text";
    text_marker.id = i;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;

    text_marker.pose.position.x = obj.x;
    text_marker.pose.position.y = obj.y;
    text_marker.pose.position.z = 2.0;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    double rad_to_deg = obj.orientation * 100.0 / M_PI;

    std::ostringstream ss;

    // degree) ss << "ID : " << obj.id << "\nnOrientation : " << std::fixed << std::setprecision(1) << rad_to_deg << "°";
    // rad) ss << "ID : " << obj.id << "\nOrientation : " << std::fixed << std::setprecision(2) << obj.orientation;
    ss << "ID : " << obj.id << "\nOrientation : " << std::fixed << std::setprecision(2) << obj.orientation;
    text_marker.text = ss.str();
    text_marker.lifetime = ros::Duration(0.5);

    marker_array.markers.push_back(text_marker);
    }

    marker_pub.publish(marker_array);
}

void RVIZ_FILTER::sdsm_callback(const j3224_msgs::sdsm::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for (size_t i = 0; i < msg->objects.size(); ++i)
    {
        const auto &obj = msg->objects[i];
        const auto &detObj = obj.detObjCommon;
        
        // SDSM refPos는 이미 EPSG:5179 좌표계 (east, north)
        double ref_latitude = msg->refPos.latitude * 1e-7;   // EPSG:5179 easting (실제로는 latitude 필드에 저장)
        double ref_longitude = msg->refPos.longitude * 1e-7; // EPSG:5179 northing (실제로는 longitude 필드에 저장)
        
        double ref_east = 0.0;
        double ref_north = 0.0;
        wgs84_to_epsg5179(ref_latitude, ref_longitude, ref_north, ref_east);
        // ROS_INFO("%lf %lf : %lf %lf", ref_latitude, ref_longitude, ref_east, ref_north);
        // TODO: ref의 heading 정보 필요 (SDSM 구조체에서 확인 필요)
        // 임시로 0도(북쪽)를 기준으로 설정
        double ref_heading = 0.0; // 라디안, ref가 바라보는 방향
        
        // 로컬 좌표계 (ref 기준 front-left) -> 글로벌 좌표계 (east-north) 변환
        double local_x = detObj.offsetX * 0.01; // cm to m, ref 기준 전방(+x)
        double local_y = detObj.offsetY * 0.01; // cm to m, ref 기준 좌측(+y)
        double local_z = detObj.offsetZ * 0.01; // cm to m
        
        // 회전 변환: 로컬 -> 글로벌
        double cos_heading = std::cos(ref_heading);
        double sin_heading = std::sin(ref_heading);
        
        double global_x_offset = local_x * cos_heading - local_y * sin_heading;
        double global_y_offset = local_x * sin_heading + local_y * cos_heading;
        
        // 절대좌표 계산
        double abs_east = ref_east + global_x_offset;
        double abs_north = ref_north + global_y_offset;
        
        // host 좌표를 기준으로 한 상대좌표로 변환 (ego_frame 기준)
        double obj_x, obj_y;
        if (host_initialized_) {
            obj_x = abs_east - host_east_;
            obj_y = abs_north - host_north_;
        } else {
            ROS_WARN("Host position not initialized, using absolute coordinates");
            obj_x = abs_east;
            obj_y = abs_north;
        }
        
        // 속도로부터 heading 계산 (SDSM의 heading은 0.0125도 단위)
        double heading_rad = detObj.heading * 0.0125 * M_PI / 180.0; // 0.0125도 단위를 라디안으로 변환

        // =========================== 큐브 마커 (객체 표현) ===========================
        visualization_msgs::Marker cube_marker;
        
        cube_marker.header.frame_id = "ego_frame";
        cube_marker.header.stamp = now;
        cube_marker.ns = "sdsm_objects";
        cube_marker.id = 200 + i;
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;

        // 위치 설정
        cube_marker.pose.position.x = obj_x;
        cube_marker.pose.position.y = obj_y;
        cube_marker.pose.position.z = 0.0;//local_z + 0.5; // 지면에서 약간 위로
        
        // 방향 설정
        getOrientationFromDirection(cube_marker, heading_rad);
        
        // 객체 타입에 따른 크기 설정
        switch(detObj.objType)
        {
            case 0: // unknown
                cube_marker.scale.x = 2.0;
                cube_marker.scale.y = 2.0;
                cube_marker.scale.z = 1.5;
                break;
            case 1: // vehicle
                cube_marker.scale.x = 4.5;
                cube_marker.scale.y = 2.0;
                cube_marker.scale.z = 1.5;
                break;
            case 2: // vru (vulnerable road user)
                cube_marker.scale.x = 0.8;
                cube_marker.scale.y = 0.8;
                cube_marker.scale.z = 1.8;
                break;
            default:
                cube_marker.scale.x = 2.0;
                cube_marker.scale.y = 2.0;
                cube_marker.scale.z = 1.5;
                break;
        }

        // 객체 타입에 따른 색상 설정
        switch(detObj.objType)
        {
            case 0: // unknown - 회색
                cube_marker.color.r = 0.5;
                cube_marker.color.g = 0.5;
                cube_marker.color.b = 0.5;
                cube_marker.color.a = 0.8;
                break;
            case 1: // vehicle - 빨간색
                cube_marker.color.r = 1.0;
                cube_marker.color.g = 0.0;
                cube_marker.color.b = 0.0;
                cube_marker.color.a = 0.8;
                break;
            case 2: // vru - 주황색
                cube_marker.color.r = 1.0;
                cube_marker.color.g = 0.5;
                cube_marker.color.b = 0.0;
                cube_marker.color.a = 0.8;
                break;
            default:
                cube_marker.color.r = 0.0;
                cube_marker.color.g = 1.0;
                cube_marker.color.b = 1.0;
                cube_marker.color.a = 0.8;
                break;
        }

        cube_marker.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back(cube_marker);
    }

    // 마커 배열 발행
    if (!marker_array.markers.empty()) 
    {
        marker_pub.publish(marker_array);
        // ROS_INFO("Published %lu    SDSM markers", marker_array.markers.size());
    }
}

void RVIZ_FILTER::local_callback(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg)
{
    host_east_ = msg->host_east;
    host_north_ = msg->host_north;
    host_initialized_ = true;
    
}

bool RVIZ_FILTER::wgs84_to_epsg5179(double lat_deg, double lon_deg, double& x_out, double& y_out) 
{
    PJ_CONTEXT *C;
    PJ *P;
    PJ_COORD coord;

    C = proj_context_create();

    // WGS84 → EPSG:5179
    P = proj_create_crs_to_crs(C,
                               "EPSG:4326",     // 입력 좌표계 (WGS84)
                               "EPSG:5179",     // 출력 좌표계 (UTM-K)
                               NULL);

    if (P == nullptr) {
        ROS_ERROR("Failed to create coordinate transformation.");
        return false;
    }

    coord = proj_coord(lat_deg, lon_deg, 0, 0);  // WGS84는 (lon, lat) 순서
    PJ_COORD result = proj_trans(P, PJ_FWD, coord);

    x_out = result.xy.x;
    y_out = result.xy.y;

    proj_destroy(P);
    proj_context_destroy(C);

    return true;
}

void RVIZ_FILTER::loop(void)
{

}

