// katech_ped_detector.cpp
// ---------------------------------------------------------------------------
// katech_ped_detector.py 의 C++ 포팅 (ped-detector-cpp 하네스, 01_spec.md).
//
// 원본 Python 검출 로직(횡단보도 보행자 on_crosswalk)을 동등 동작으로 이식하되,
// 검출 파이프라인에 방향 게이트를 추가한다. (크기 게이트 제거 — 사용자 요청: 진행방향만 고려)
//   필터 순서: ① 타입{1,2}(이전 버전) → ② 횡단보도 멤버십(LINK 게이팅 + ray-casting)
//              → ③ 방향(이동 객체만, 절대속도가 길이축과 이루는 각 ≤ 40°)
//
// 토픽/메시지/좌표계/CW 값/occupancy 산출식은 Python 과 동일(신규는 size·direction 게이트뿐).
//   sub: /localization/to_control_team (mmc_msgs/to_control_team_from_local_msg)
//        /track_Multi_RS               (perception_ros_msg/object_array_msg)
//   pub: /katech_msg/crosswalk_detection (katech_custom_msgs/ped_crosswalk_check_array_msg)
//        /katech_msg/crosswalk_occupancy (katech_custom_msgs/crosswalk_occupancy_msg)
//
// CAN 미접촉 — canlib/kvaDbLib/can_pub_func 링크 없음.
// ---------------------------------------------------------------------------

#include <ros/ros.h>

#include <cmath>
#include <map>
#include <set>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdint>

#include <mmc_msgs/to_control_team_from_local_msg.h>
#include <perception_ros_msg/object_array_msg.h>
#include <perception_ros_msg/object_msg.h>
#include <katech_custom_msgs/ped_crosswalk_check_msg.h>
#include <katech_custom_msgs/ped_crosswalk_check_array_msg.h>
#include <katech_custom_msgs/crosswalk_occupancy_msg.h>

// ===========================================================================
// 단일 횡단보도: 좌표(EPSG:5179) + bbox + PCA major axis(사전계산)
// ===========================================================================
struct Crosswalk {
  int id;
  std::vector<std::array<double, 2>> coords;  // (east, north)
  double min_x, max_x, min_y, max_y;          // bbox (Python Crosswalk.__init__ 동일)
  double axis_x, axis_y;                       // PCA major axis(단위벡터), 부호 무의미

  Crosswalk(int cid, const std::vector<std::array<double, 2>>& c)
      : id(cid), coords(c) {
    // --- bbox ---
    min_x = max_x = coords[0][0];
    min_y = max_y = coords[0][1];
    for (const auto& p : coords) {
      min_x = std::min(min_x, p[0]);
      max_x = std::max(max_x, p[0]);
      min_y = std::min(min_y, p[1]);
      max_y = std::max(max_y, p[1]);
    }
    computePcaAxis();
  }

  // point-in-polygon (bbox 조기배제 + ray-casting), Python point_in_rectangle 등가
  bool pointInPoly(double px, double py) const {
    if (!(min_x <= px && px <= max_x && min_y <= py && py <= max_y)) return false;
    const int n = static_cast<int>(coords.size());
    bool inside = false;
    int j = n - 1;
    for (int i = 0; i < n; ++i) {
      const double xi = coords[i][0], yi = coords[i][1];
      const double xj = coords[j][0], yj = coords[j][1];
      if (((yi > py) != (yj > py)) &&
          (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
      j = i;
    }
    return inside;
  }

 private:
  // §4.4 PCA major axis. 정점을 등가중 점집합으로 보고 2x2 공분산 최대 고유벡터.
  void computePcaAxis() {
    const int n = static_cast<int>(coords.size());
    double mx = 0.0, my = 0.0;
    for (const auto& p : coords) { mx += p[0]; my += p[1]; }
    mx /= n; my /= n;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (const auto& p : coords) {
      const double dx = p[0] - mx, dy = p[1] - my;
      sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
    }
    sxx /= n; syy /= n; sxy /= n;

    const double tr = sxx + syy;
    const double det = sxx * syy - sxy * sxy;
    const double disc = std::sqrt(std::max(0.0, (tr / 2.0) * (tr / 2.0) - det));
    const double l1 = tr / 2.0 + disc;  // 최대 고유값

    double ex, ey;
    if (std::fabs(sxy) > 1e-12) {
      ex = l1 - syy; ey = sxy;
    } else {
      if (sxx >= syy) { ex = 1.0; ey = 0.0; }
      else            { ex = 0.0; ey = 1.0; }
    }
    const double norm = std::hypot(ex, ey);
    axis_x = ex / norm;
    axis_y = ey / norm;
  }
};

// ===========================================================================
// 검출 노드
// ===========================================================================
class CrosswalkDetector {
 public:
  CrosswalkDetector();

  void hostCb(const mmc_msgs::to_control_team_from_local_msg& m);
  void objCb(const perception_ros_msg::object_array_msg& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber host_sub_, obj_sub_;
  ros::Publisher  det_pub_, occ_pub_;

  // --- 튜닝 상수 (§4.6, ros::param 로 override 가능) ---
  double ped_size_max_;        // PED_SIZE_MAX          = 2.0  [m]
  double move_min_speed_;      // MOVE_MIN_SPEED        = 0.3  [m/s]
  double cross_angle_max_deg_; // CROSS_ANGLE_MAX_DEG   = 40   [deg]
  double cross_cos_;           // = cos(cross_angle_max_deg_)  (파생)
  double ego_vel_ema_alpha_;   // EGO_VEL_EMA_ALPHA     = 0.3
  double ego_vel_dt_min_;      // EGO_VEL_DT_MIN        = 1e-3 [s]
  double ego_vel_dt_max_;      // EGO_VEL_DT_MAX        = 1.0  [s]

  // --- 크로스워크 데이터 ---
  std::vector<Crosswalk> crosswalks_;   // id 1..18 순서
  std::map<int, std::set<int>> cw_links_;  // 크로스워크 N -> 접근 LINK_ID 집합

  // --- 자차 상태 ---
  double host_east_  = 0.0;
  double host_north_ = 0.0;
  double host_yaw_   = 0.0;
  int    host_link_id_ = 0;
  bool   host_data_updated_ = false;

  // --- ego 절대속도 유한차분 상태 (§4.3) ---
  double prev_e_ = 0.0, prev_n_ = 0.0;
  ros::Time prev_t_;
  bool   have_prev_ = false;
  double v_ego_e_ = 0.0, v_ego_n_ = 0.0;
  bool   ego_vel_valid_ = false;

  void initCrosswalks();
  void initLinks();
  // 절대좌표(map)가 어느 크로스워크에 속하는지: LINK 게이팅 + ray-cast. 없으면 -1.
  int  membershipCrosswalk(double abs_e, double abs_n) const;
};

CrosswalkDetector::CrosswalkDetector() {
  ros::NodeHandle pnh("~");
  pnh.param("ped_size_max",        ped_size_max_,        2.0);
  pnh.param("move_min_speed",      move_min_speed_,      0.3);
  pnh.param("cross_angle_max_deg", cross_angle_max_deg_, 40.0);
  pnh.param("ego_vel_ema_alpha",   ego_vel_ema_alpha_,   0.3);
  pnh.param("ego_vel_dt_min",      ego_vel_dt_min_,      1e-3);
  pnh.param("ego_vel_dt_max",      ego_vel_dt_max_,      1.0);
  cross_cos_ = std::cos(cross_angle_max_deg_ * M_PI / 180.0);  // cos(40°)=0.766044

  initCrosswalks();
  initLinks();

  host_sub_ = nh_.subscribe("/localization/to_control_team", 10,
                            &CrosswalkDetector::hostCb, this);
  obj_sub_  = nh_.subscribe("/track_Multi_RS", 10,
                            &CrosswalkDetector::objCb, this);

  det_pub_ = nh_.advertise<katech_custom_msgs::ped_crosswalk_check_array_msg>(
      "/katech_msg/crosswalk_detection", 10);
  occ_pub_ = nh_.advertise<katech_custom_msgs::crosswalk_occupancy_msg>(
      "/katech_msg/crosswalk_occupancy", 10);
}

// crosswalk_data (1~18, EPSG:5179 폴리곤) — katech_ped_detector.py 값 그대로.
void CrosswalkDetector::initCrosswalks() {
  const std::map<int, std::vector<std::array<double, 2>>> crosswalk_data = {
    {1, {{930819.312725,1929593.158143},{930807.530293,1929580.608639},{930804.537889,1929582.883314},{930803.754045,1929582.693963},{930800.736230,1929585.001303},{930798.147151,1929585.252169},{930814.166390,1929602.326491},{930814.430028,1929599.606956},{930813.356668,1929598.462133},{930816.299566,1929596.210116},{930816.286256,1929595.495250}}},
    {2, {{930819.933061,1929617.498679},{930817.484334,1929614.491229},{930817.695865,1929613.842349},{930815.283504,1929610.895521},{930813.983369,1929609.298809},{930788.698959,1929628.204441},{930789.932348,1929629.623349},{930790.729092,1929629.069490},{930793.150823,1929631.994565},{930792.937083,1929632.723870},{930795.460264,1929635.651967}}},
    {3, {{931618.480129,1928668.245223},{931602.430990,1928680.837104},{931604.756398,1928683.910106},{931604.535801,1928684.329460},{931606.966005,1928687.602070},{931606.359503,1928688.086279},{931607.641929,1928689.732141},{931625.739121,1928675.649309},{931624.307032,1928673.985063},{931623.131051,1928674.888081},{931620.632328,1928671.712353},{931620.838390,1928671.200786}}},
    {4, {{931746.802920,1928832.583676},{931731.083283,1928844.947864},{931732.751645,1928847.093707},{931731.718782,1928848.551824},{931733.372971,1928850.708445},{931731.845245,1928851.916781},{931731.955025,1928853.249105},{931733.455832,1928853.860780},{931753.293170,1928838.226088},{931753.572679,1928835.924257},{931752.549467,1928835.643084},{931750.919243,1928836.947748},{931749.116990,1928834.923729},{931748.438488,1928834.717009}}},
    {5, {{931584.965262,1928973.697114},{931581.771173,1928976.155563},{931581.323971,1928975.925517},{931578.234135,1928978.366626},{931585.070821,1928987.307505},{931587.654537,1928990.733507},{931592.420497,1928996.805367},{931595.503240,1928994.370156},{931596.434902,1928995.316672},{931599.639435,1928992.892383},{931593.808205,1928985.349048},{931591.235266,1928981.811252}}},
    {6, {{931483.574338,1929056.144024},{931480.594260,1929058.534888},{931479.977918,1929058.443113},{931477.062238,1929060.808009},{931476.218800,1929059.782930},{931473.932140,1929060.184437},{931490.058985,1929080.188544},{931491.167411,1929078.441717},{931490.336670,1929077.309865},{931493.266962,1929074.906220},{931493.902490,1929075.049275},{931496.819689,1929072.651420}}},
    {7, {{931280.158279,1929216.475039},{931277.153107,1929218.935474},{931276.793565,1929218.787803},{931273.626088,1929221.207678},{931272.885465,1929220.381162},{931270.311425,1929220.938055},{931285.571652,1929240.179616},{931287.512710,1929238.690955},{931286.833061,1929237.801428},{931289.877390,1929235.393598},{931290.379072,1929235.592503},{931293.393199,1929233.197194}}},
    {8, {{931109.745208,1929350.398237},{931107.517502,1929352.183155},{931106.537442,1929352.114020},{931104.362517,1929353.968414},{931102.217096,1929355.490024},{931123.572270,1929370.300755},{931122.889192,1929366.812558},{931125.221841,1929364.919769},{931126.143806,1929365.194742},{931128.402187,1929363.248166}}},
    {9, {{930949.468998,1929473.449621},{930947.294577,1929475.114827},{930945.901475,1929474.049849},{930943.694736,1929475.722102},{930941.696300,1929476.343399},{930953.418371,1929492.148083},{930954.726980,1929490.412941},{930956.895444,1929488.757050},{930957.668540,1929488.887335},{930959.876393,1929487.248217}}},
    {10, {{931009.892314,1929869.347714},{930989.422158,1929884.718851},{930990.840403,1929886.596273},{930992.370996,1929885.438507},{930994.597744,1929888.430465},{930993.428509,1929889.951139},{930995.704454,1929892.954529},{931015.198660,1929878.182050},{931012.906789,1929875.190725},{931013.102253,1929874.469843},{931010.822240,1929871.488405},{931011.260692,1929871.156762}}},
    {11, {{931212.263960,1930136.355868},{931193.918693,1930150.708746},{931196.209713,1930153.726499},{931195.138942,1930155.165075},{931197.449529,1930158.192332},{931196.977609,1930158.562419},{931198.259128,1930160.208038},{931218.954002,1930144.040402},{931217.640006,1930142.358489},{931216.704028,1930143.092506},{931214.404972,1930140.144679},{931214.573071,1930139.397542}}},
    {12, {{931400.789431,1930377.100777},{931377.662387,1930395.166111},{931388.157495,1930408.808547},{931411.249244,1930390.865166}}},
    {13, {{931566.243088,1930606.958922},{931550.328599,1930618.891965},{931552.605269,1930621.890547},{931551.541686,1930623.327522},{931553.806946,1930626.322392},{931553.472129,1930626.575649},{931554.687866,1930628.205072},{931573.001803,1930614.490055},{931571.785671,1930612.843513},{931570.606114,1930613.719660},{931568.344859,1930610.730212},{931568.518059,1930609.988166}}},
    {14, {{931352.846202,1930782.826228},{931349.858383,1930785.122693},{931349.131475,1930784.979381},{931346.109525,1930787.268650},{931345.565184,1930786.559611},{931343.954436,1930787.820737},{931360.219697,1930809.253150},{931361.251073,1930807.147970},{931359.631316,1930805.055679},{931362.667769,1930802.767926},{931364.070372,1930803.826035},{931367.075453,1930801.530354}}},
    {15, {{931152.906209,1930951.869593},{931149.849024,1930954.184512},{931149.036778,1930954.088520},{931146.069882,1930956.350450},{931145.845868,1930956.166210},{931144.124700,1930957.392427},{931153.298176,1930965.321247},{931154.671952,1930963.835662},{931153.624067,1930962.961196},{931156.674913,1930960.641370},{931157.489833,1930960.738407},{931160.532077,1930958.481263}}},
    {16, {{931145.949481,1930939.625515},{931142.875002,1930941.936252},{931142.241202,1930941.683648},{931139.166345,1930944.007382},{931138.950054,1930943.606514},{931137.240376,1930944.604723},{931142.099591,1930954.803818},{931143.836682,1930953.677868},{931143.508956,1930953.037144},{931146.613575,1930950.765446},{931147.201312,1930950.994365},{931150.315106,1930948.674230}}},
    {17, {{931150.628541,1930993.535675},{931121.428225,1931013.690255},{931123.048078,1931016.201683},{931123.925548,1931015.607832},{931126.084990,1931018.699356},{931125.898143,1931019.462050},{931128.040561,1931022.529483},{931155.890378,1931003.177928},{931153.792914,1931000.150670},{931153.963149,1930999.391808},{931151.830824,1930996.346387},{931152.275721,1930996.057667}}},
    {18, {{931590.186643,1931656.769042},{931587.251728,1931657.481834},{931585.930903,1931656.116009},{931582.747422,1931656.855280},{931580.076216,1931657.388505},{931594.432587,1931675.332380},{931595.254946,1931672.690294},{931593.559388,1931670.413925},{931596.258215,1931669.702358},{931595.873220,1931668.205892},{931598.831104,1931667.463536}}},
  };
  crosswalks_.clear();
  crosswalks_.reserve(crosswalk_data.size());
  for (const auto& kv : crosswalk_data) {         // std::map -> id 오름차순
    crosswalks_.emplace_back(kv.first, kv.second);
  }
}

// CW_LINKS (크로스워크 N -> 접근 LINK_ID 집합) — katech_ped_detector.py 값 그대로.
// 1~9 는 crosswalk_position.md §145/§155 원문. 10~18 은 md 에 링크표가 없어 senario mat 에서
// 도출(정지선 링크 is_stop_line=1 + 리드거리 20m 확보용 선행 링크; 동일 규칙으로 1~9 재현 확인).
// #15/#16 은 같은 코너의 스태거드 쌍이라 접근 링크 661/662 를 공유(폴리곤은 비중첩).
void CrosswalkDetector::initLinks() {
  cw_links_ = {
    {1, {1239, 1238, 1242, 1241}},
    {2, {1205}},
    {3, {465, 467, 463}},
    {4, {417}},
    {5, {1029, 1025, 1027}},
    {6, {1092, 1094, 1090, 1091, 1093, 1089}},
    {7, {1370, 1368, 1372, 1369, 1367, 1371}},
    {8, {1326, 1325, 1330, 1331}},
    {9, {1257, 1258}},
    {10, {870, 871, 877}},
    {11, {1163, 1164, 1165, 1166, 1167, 1971}},
    {12, {1080, 1082, 1084, 1086}},
    {13, {876, 953}},
    {14, {574, 579, 580}},
    {15, {661, 662}},
    {16, {661, 662}},
    {17, {787}},
    {18, {2086, 2206, 2374}},
  };
}

int CrosswalkDetector::membershipCrosswalk(double abs_e, double abs_n) const {
  // 전체 18개 순회, host_link_id ∈ CW_LINKS[N] 인 것만 ray-cast.
  // 폴리곤끼리 겹치지 않으므로(#15/#16 도 별개 영역) 결과는 0/1개.
  for (const auto& cw : crosswalks_) {
    auto it = cw_links_.find(cw.id);
    if (it == cw_links_.end()) continue;
    if (it->second.count(host_link_id_) == 0) continue;  // LINK 게이팅
    if (cw.pointInPoly(abs_e, abs_n)) return cw.id;
  }
  return -1;
}

// 자차 상태 콜백 + ego 절대속도 유한차분 (§4.3)
void CrosswalkDetector::hostCb(const mmc_msgs::to_control_team_from_local_msg& m) {
  const ros::Time t = ros::Time::now();  // 벽시계(msg.time 미의존)
  if (have_prev_) {
    const double dt = (t - prev_t_).toSec();
    if (dt >= ego_vel_dt_min_ && dt <= ego_vel_dt_max_) {
      const double raw_e = (m.host_east  - prev_e_) / dt;
      const double raw_n = (m.host_north - prev_n_) / dt;
      const double a = ego_vel_ema_alpha_;
      v_ego_e_ = a * raw_e + (1.0 - a) * v_ego_e_;   // EMA
      v_ego_n_ = a * raw_n + (1.0 - a) * v_ego_n_;
      ego_vel_valid_ = true;
    } else if (dt > ego_vel_dt_max_) {
      ego_vel_valid_ = false;   // stale → 리셋(폭주 방지)
    }
    // dt < DT_MIN: 갱신 스킵(이전 v 유지)
  }
  prev_e_ = m.host_east; prev_n_ = m.host_north; prev_t_ = t; have_prev_ = true;

  host_east_  = m.host_east;
  host_north_ = m.host_north;
  host_yaw_   = m.host_yaw;
  host_link_id_ = static_cast<int>(m.LINK_ID);
  host_data_updated_ = true;
}

// 오브젝트 배열 콜백 — 검출 파이프라인(순서 준수)
void CrosswalkDetector::objCb(const perception_ros_msg::object_array_msg& msg) {
  if (!host_data_updated_) {
    ROS_WARN("자차 위치 정보가 아직 수신되지 않았습니다.");
    return;
  }

  // ① 타입 필터 — 이전 버전 복원: status ∈ {1,2}
  std::vector<const perception_ros_msg::object_msg*> type_objs;
  type_objs.reserve(msg.data.size());
  for (const auto& o : msg.data) {
    if (o.status == 1 || o.status == 2) type_objs.push_back(&o);
  }

  katech_custom_msgs::ped_crosswalk_check_array_msg arr;
  std::set<int> occ_ids;

  const double c = std::cos(host_yaw_);
  const double s = std::sin(host_yaw_);

  if (type_objs.empty()) {
    // Python: active_objects 없으면 zero-entry 1개 append (CAN writer size==1 분기)
    katech_custom_msgs::ped_crosswalk_check_msg e;
    e.id = 0; e.status = 0; e.on_crosswalk = 0; e.rel_pos_x = 0.0; e.rel_pos_y = 0.0;
    arr.data.push_back(e);
  } else {
    for (const auto* op : type_objs) {
      const perception_ros_msg::object_msg& o = *op;

      // (② 크기 게이트 제거 — 방향 게이트만 사용, 사용자 요청 "진행방향만 고려")
      // 절대좌표 (§4.1): abs = host + R(yaw)*(x,y)
      const double abs_e = host_east_  + (o.x * c - o.y * s);
      const double abs_n = host_north_ + (o.x * s + o.y * c);

      // ③ 멤버십: LINK 게이팅 + ray-cast
      const int cw = membershipCrosswalk(abs_e, abs_n);
      if (cw < 0) continue;

      // ④ 방향 게이트 (신규, 이동 객체만) — §4.3/§4.5
      //    절대속도 = R(yaw)*(vx,vy) + v_ego_map
      const double vx_map = o.vx * c - o.vy * s;
      const double vy_map = o.vx * s + o.vy * c;
      const double v_map_e = vx_map + v_ego_e_;   // ego 가산(필수)
      const double v_map_n = vy_map + v_ego_n_;
      const double sp = std::hypot(v_map_e, v_map_n);
      if (ego_vel_valid_ && sp >= move_min_speed_) {
        const double ux = v_map_e / sp, uy = v_map_n / sp;
        const int idx = cw - 1;  // crosswalks_ 는 id 오름차순 → id N == index N-1
        const double dot = ux * crosswalks_[idx].axis_x + uy * crosswalks_[idx].axis_y;
        // 사이각 ≤ 40° ⟺ |dot| ≥ cos40°. 미달 = 길이축 수직 = 통과차량 → 배제.
        if (std::fabs(dot) < cross_cos_) continue;
      }
      // sp < move_min_speed 또는 ego_vel 무효: 방향 스킵(정지 보행자 유지)

      // 통과 → 검출 엔트리(객체마다 독립) + occupancy
      katech_custom_msgs::ped_crosswalk_check_msg e;
      e.id = o.id;
      e.status = o.status;
      e.on_crosswalk = 1;
      e.rel_pos_x = o.x;
      e.rel_pos_y = o.y;
      arr.data.push_back(e);
      occ_ids.insert(cw);
    }
  }

  // 발행 1: detection
  arr.time = ros::Time::now();
  det_pub_.publish(arr);

  // 발행 2: occupancy (완전 게이트 통과분)
  katech_custom_msgs::crosswalk_occupancy_msg occ;
  occ.time = ros::Time::now();
  occ.crosswalk1_occupied = (occ_ids.count(1) > 0);
  occ.crosswalk2_occupied = (occ_ids.count(2) > 0);
  occ.occupied_ids.clear();
  occ.occupied_ids.reserve(occ_ids.size());
  for (int id : occ_ids) {  // std::set → 오름차순
    occ.occupied_ids.push_back(static_cast<uint8_t>(id));
  }
  occ_pub_.publish(occ);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "katech_ped_detector");
  CrosswalkDetector detector;
  ros::spin();
  return 0;
}
