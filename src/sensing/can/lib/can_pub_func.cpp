#include <can_pub.h>

canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
  canInitializeLibrary();
  int open_flag;

  if(init_access_flag == true){
    open_flag = canOPEN_REQUIRE_INIT_ACCESS;

  }else{
    open_flag = canOPEN_NO_INIT_ACCESS;
  }

  cout<<"Opening the channel "<<channel_num<<"..."<<endl;
  hCAN = canOpenChannel(channel_num, open_flag);

  cout<<"hCAN status = "<<hCAN<<endl;

  if(hCAN == canOK){
    cout<<"The CAN channel "<<channel_num<<" has been opened successfully..."<<endl;
  }
  
  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
  can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
  can_status = canBusOn(hCAN);

  if(can_status == canOK){
    cout<<"The CAN bus is on..."<<endl;
  }

  kvaDb_status = kvaDbOpen(&dh);
  kvaDb_status = kvaDbReadFile(dh, filename);
  kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

  if(hCAN == canOK){
    return canOK;

  }else{
    return canERR_NOMSG;
  }
    
}


void FRONT_RADAR_CAN_READER(){
  int msg_num = 32;
  vector<char*> signal_list = {(char*)"CAN_TX_TRACK_STATUS",\
                               (char*)"CAN_TX_TRACK_RANGE",\
                               (char*)"CAN_TX_TRACK_RANGE_RATE",\
                               (char*)"CAN_TX_TRACK_ANGLE"};

  vector<double> empty_vec(signal_list.size());
  vector<vector<double>> RADAR;
  int temp_substring, idx, pur;
  int ret = 0;
  
  for(int i=0; i!=msg_num; i++){
    RADAR.push_back(empty_vec);
  }
  
  ros::Rate rate(freq_for_channel_0);
  
  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);
    
    for(int temp_substring=0; temp_substring<msg_num; temp_substring++)
    {
      if(kvaDb_status == kvaDbOK){
        kvaDbGetMsgName(mh, buff, sizeof(buff));
        sscanf(buff, "Track%2d", &temp_substring);
        // if(temp_substring == 0)
        // {

        // }
        // else
        // {
        //   idx = temp_substring - 1;
        // } 
        idx = temp_substring;

        for(int i=0; i!=signal_list.size(); i++){
          kvaDbGetSignalByName(mh, signal_list[i], &sh);
          ret = kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));
          // ROS_INFO("WHERE %s   %d", buff, idx);
          
          RADAR[idx][i] = value;
          
        }
      }
      pur = temp_substring;
    }

    if((pur == msg_num - 1) && (ret == 0))
    {
      mmc_msgs::object_array_msg msg;
      msg.time = ros::Time::now();

      for(int i=0; i!=msg_num; i++){
        mmc_msgs::object_msg temp_msg;
        
        double status     = RADAR[i][0];
        double range      = RADAR[i][1];
        double range_rate = RADAR[i][2];
        double angle      = RADAR[i][3] * 0.0174533; // PI/180 = 0.0174533, rad

        temp_msg.id = i;
        temp_msg.x  = range * cos(-angle);
        temp_msg.y  = range * sin(-angle);
        temp_msg.vx = range_rate * cos(-angle);
        temp_msg.vy = range_rate * sin(-angle);

        msg.data.push_back(temp_msg);
      }

      radar_pub.publish(msg);
      temp_substring = 0;
      RADAR.clear();
      for(int i=0; i!=msg_num; i++){
        RADAR.push_back(empty_vec);
      }
    }
    
    rate.sleep();
  }
}


void FRONT_LIDAR_CAN_READER(){
  vector<tuple<char*, vector<char*>>> msg_list;

  msg_list.push_back(make_tuple((char*)"LUX_ObjectDataListHeader", vector<char*> {(char*)"None"}));

  msg_list.push_back(make_tuple((char*)"LUX_ObjectDataListTailer", vector<char*> {(char*)"NumberOfObjectSent"}));

  msg_list.push_back(make_tuple((char*)"LUXObjectDataTracking1",   vector<char*> {(char*)"Object_ID",\
                                                                                  (char*)"Position_X",\
                                                                                  (char*)"Position_Y",\
                                                                                  (char*)"Velocity_X",\
                                                                                  (char*)"Velocity_Y"}));

  msg_list.push_back(make_tuple((char*)"LUXObjectDataBox",         vector<char*> {(char*)"ObjectID",\
                                                                                  (char*)"BoxSize_X_length",\
                                                                                  (char*)"BoxSize_Y_width",\
                                                                                  (char*)"BoxOrientation"}));

  msg_list.push_back(make_tuple((char*)"VehicleVelocity",          vector<char*> {(char*)"Velocity"}));

  bool header_flag = false;
  bool id_matched_flag = false;
  int msg_idx = 0;
  int obj_id = 0;
  float ego_vel = 0;
  mmc_msgs::object_array_msg msg;
  mmc_msgs::object_msg temp_msg;

  ros::Rate rate(freq_for_channel_1);

  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_1); // need to check the timeout value
    kvaDbGetMsgById(dh, temp_id, &mh);
    kvaDbGetMsgName(mh, buff, sizeof(buff));
    bool matched_flag = false;
    int data_info_flag = 1; // represents type of velocity(absolute or relative, refer to the database file)

    for(int i=0; i!=msg_list.size(); i++){
        
      if(strcmp(buff, get<0>(msg_list[i])) == 0){
        matched_flag = true;
        msg_idx = i;
      }
    }

    if(matched_flag == false){
      msg_idx = 0;
    }

    if(matched_flag == true){
      
      switch(msg_idx){

        case 0: // "LUX_ObjectDataListHeader"
          msg.time = ros::Time::now();
          header_flag = true;
          kvaDbGetSignalByName(mh, (char*)"ObjectDataInfoFlag", &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));
          data_info_flag = value;
          temp_msg.status = value;
        break;

        case 1: // "LUX_ObjectDataListTailer"

          if(header_flag == true){
            pub1.publish(msg);
            msg.data.clear();
          }

          header_flag = false;
        break;

        case 2: // "LUXObjectDataTracking1"

          for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
            kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

            switch(i){

              case 0: // Object_ID
                temp_msg.id = (int)value;
                obj_id = (int)value;
              break; 

              case 1: // Position_X
                temp_msg.x = value * 0.01 - 3.85; // meter, to base_link
              break;

              case 2: // Position_Y
                temp_msg.y = value * 0.01; // meter
              break;

              case 3: // Velocity_X

                if(data_info_flag == 0 || data_info_flag == 2){ // 속도가 절대속도를 나타내고 있는 경우 상대속도로 변환해준다
                  temp_msg.vx = value - ego_vel;

                }else if(data_info_flag == 1 || data_info_flag == 3){ // 속도가 상대속도를 나타내고 있는 경우
                  temp_msg.vx = value;
                }

                temp_msg.vx = value; // m/s
              break;

              case 4: // Velocity_Y
                temp_msg.vy = value;
                // temp_msg.vy = 0;
                msg.data.push_back(temp_msg);
                temp_msg.x = 0;
                temp_msg.y = 0;
                temp_msg.vx = 0;
                temp_msg.vy = 0;
              break;
            }
          }
        break;
          
        case 3: // "LUXObjectDataBox"

          for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
            kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

            switch(i){

              case 0: // ObjectID

                if((int)value != obj_id){
                  id_matched_flag = false;
                  
                }else{
                  id_matched_flag = true;
                }

              break;

              case 1: // BoxSize_X_length

                if(id_matched_flag == true){
                  msg.data.back().size_x = value * 0.01; // meter
                }

              break;

              case 2: // BoxSize_Y_width

                if(id_matched_flag == true){
                  msg.data.back().size_y = value * 0.01; // meter
                }

              break;

              case 3: // BoxOrientation

                if(id_matched_flag == true){
                  msg.data.back().orientation = value; // need to check the unit
                }
              break;
            }
          }
        break;

        case 4: // "VehicleVelocity"
          kvaDbGetSignalByName(mh, (char*)"VehicleVelocity", &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));
          ego_vel = value;
        break;
      }
    }

    rate.sleep();
  }
}


void VISION_CAN_READER(){
  vector<tuple<char*, vector<char*>>> msg_list_lane;
  vector<tuple<char*, vector<char*>>> msg_list_obj;

  // lane
  msg_list_lane.push_back(make_tuple((char*)"LD_Left_Lane_A",  vector<char*> {(char*)"Lh_LaneMarkType",\
                                                                              (char*)"Lh_LaneMarkQuality",\
                                                                              (char*)"Lh_LaneMarkPosition",\
                                                                              (char*)"Lh_LaneMarkModelA"}));

  msg_list_lane.push_back(make_tuple((char*)"LD_Left_Lane_B",  vector<char*> {(char*)"Lh_LaneMarkViewRange",\
                                                                              (char*)"Lh_LaneMarkModelDerivA",\
                                                                              (char*)"Lh_LaneMarkHeadAngle"}));

  msg_list_lane.push_back(make_tuple((char*)"LD_Right_Lane_A", vector<char*> {(char*)"Rh_LaneMarkType",\
                                                                              (char*)"Rh_LaneMarkQuality",\
                                                                              (char*)"Rh_LaneMarkPosition",\
                                                                              (char*)"Rh_LaneMarkModelA"}));

  msg_list_lane.push_back(make_tuple((char*)"LD_Right_Lane_B", vector<char*> {(char*)"Rh_LaneMarkViewRange",\
                                                                              (char*)"Rh_LaneMarkModelDerivA",\
                                                                              (char*)"Rh_LaneMarkHeadAngle"}));

  // object
  msg_list_obj.push_back(make_tuple((char*)"Obstacle_Data_A_", vector<char*> {(char*)"ObjectIdentifier_A_"}));
  
  msg_list_obj.push_back(make_tuple((char*)"Obstacle_Data_B_", vector<char*> {(char*)"Range_B_",\
                                                                              (char*)"ObjectValidity_B_",\
                                                                              (char*)"RangeRate_B_",\
                                                                              (char*)"ObjectPostionY_B_"}));

  msg_list_obj.push_back(make_tuple((char*)"Obstacle_Data_C_", vector<char*> {(char*)"ObjectType_C_"}));

  bool matched_flag_lane = false;
  bool matched_flag_obj = false;
  int msg_idx_lane = 0;
  int msg_idx_obj[2] = {0,0};
  int msg_num_obj;
  int obj_num = 8;
  double lane_width;
  double lane_compensation;
  char temp[1];
  char sliced_buff_char[20];
  vector<bool> validity_flag(obj_num, true);
  vector<mmc_msgs::object_msg> temp_msg_obj(obj_num);
  mmc_msgs::object_array_msg msg_obj;
  mmc_msgs::lane_array_msg msg_lane;
  mmc_msgs::lane_msg temp_msg_lane;


  for(int i=0; i!=2; i++){
    msg_lane.data.push_back(temp_msg_lane); // msg_lane.data[0]: left lane, msg_lane.data[1]: right lane
  }
  
  ros::Time last_receive_time = ros::Time::now();

  ros::Rate rate(freq_for_channel_0);
  
  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);

    // if(ros::Time::now()-last_receive_time > ros::Duration(0.03)){
    //   cout<<"!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#!@#"<<endl;
    // }
    

    if(kvaDb_status == kvaDbOK){
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      matched_flag_lane = false;
      matched_flag_obj = false;

      last_receive_time = ros::Time::now();
      
      for(int i=0; i!=msg_list_lane.size(); i++){
          
        if(strcmp(buff, get<0>(msg_list_lane[i])) == 0){
          matched_flag_lane = true;
          msg_idx_lane = i;
        }
      }

      if(matched_flag_lane == false){
        msg_idx_lane = 0;
        // cout<<buff<<endl;
        
        string sliced_buff_str = string(&buff[0], &buff[sizeof("Obstacle_Data_A_") - 1]);
        strcpy(sliced_buff_char, sliced_buff_str.c_str());
        
        for(int i=0; i!=msg_list_obj.size(); i++){

          if(strcmp(sliced_buff_char, get<0>(msg_list_obj[i])) == 0){
            matched_flag_obj = true;
            sscanf(buff, "Obstacle_Data_%c_%d", temp, &msg_num_obj);
            msg_idx_obj[0] = i; 
            msg_idx_obj[1] = msg_num_obj - 1;
            // msg_idx_obj = {0,0} means that message = "Obstacle_Data_A_1"
            // msg_idx_obj = {1,5} means that message = "Obstacle_Data_B_6" 
            // msg_idx_obj = {2,3} means that message = "Obstacle_Data_C_4"
          }
        }
        memset(sliced_buff_char, 0, sizeof(sliced_buff_char));
      }
      
      if(matched_flag_lane == true){ // Lane
        // the normal sequence of incoming lane messages is "LD_Left_Lane_A" -> "LD_Left_Lane_B" -> "LD_Right_Lane_B" -> "LD_Right_Lane_A"
        switch(msg_idx_lane){

          case(0): // LD_Left_Lane_A

            for(int i=0; i!=get<1>(msg_list_lane[msg_idx_lane]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list_lane[msg_idx_lane])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // Lh_LaneMarkType
                  msg_lane.data[0].Lane_Name = "Left_Lane";
                  msg_lane.data[0].marker_type = (int)value;
                  msg_lane.time = ros::Time::now();
                break;

                case(1): // Lh_LaneMarkQuality
                  msg_lane.data[0].Quality = (int)value;
                break;

                case(2): // Lh_LaneMarkPosition
                  msg_lane.data[0].d = -value;
                break;

                case(3): // Lh_LaneMarkModelA
                  msg_lane.data[0].b = -value;
                break;
              }
            }
          break;

          case(1): // LD_Left_Lane_B 

            for(int i=0; i!=get<1>(msg_list_lane[msg_idx_lane]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list_lane[msg_idx_lane])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // Lh_LaneMarkViewRange
                  msg_lane.data[0].View_Range = value;
                break;

                case(1): // Lh_LaneMarkModelDerivA
                  msg_lane.data[0].a = -value;
                break;

                case(2): // Lh_LaneMarkHeadAngle
                  msg_lane.data[0].c = -value;
                break;
              }
            }
          break;

          case(2): // LD_Right_Lane_A

            for(int i=0; i!=get<1>(msg_list_lane[msg_idx_lane]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list_lane[msg_idx_lane])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // Rh_LaneMarkType
                  msg_lane.data[1].Lane_Name = "Right_Lane";
                  msg_lane.data[1].marker_type = (int)value;
                break;

                case(1): // Rh_LaneMarkQuality
                  msg_lane.data[1].Quality = (int)value;
                break;

                case(2): // Rh_LaneMarkPosition
                  msg_lane.data[1].d = -value;

                  if(msg_lane.data[0].d == 0){ // if the message "LD_Left_Lane_A" hasn't been arrived yet
                    lane_width = abs(value) + 1.75;
                    msg_lane.data[0].Lane_Name = "Left_Lane";
                    msg_lane.data[0].Quality = 0;
                    msg_lane.data[0].d = 1.75;
                    msg_lane.time = ros::Time::now();

                  }else{ 
                    lane_width = abs(msg_lane.data[0].d) + abs(msg_lane.data[1].d);
                  }

                  lane_compensation = lane_width * 0.05; // an emphirical compensation. The lane looks narrower than it looks.
                  msg_lane.data[1].d -= lane_compensation;  // right lane 
                  msg_lane.data[0].d += lane_compensation;  // left lane
                break;

                case(3): // Rh_LaneMarkModelA
                  msg_lane.data[1].b = -value;

                  pub1.publish(msg_lane);

                  // initializing the lane message topic
                  msg_lane.data.clear();

                  for(int j=0; j!=2; j++){
                    msg_lane.data.push_back(temp_msg_lane);
                  }

                break;
              }
            }
          break;

          case(3): // LD_Right_Lane_B
            for(int i=0; i!=get<1>(msg_list_lane[msg_idx_lane]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list_lane[msg_idx_lane])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // Rh_LaneMarkViewRange
                  msg_lane.data[1].View_Range = value;
                break;

                case(1): // Rh_LaneMarkModelDerivA
                  msg_lane.data[1].a = -value;
                break;

                case(2): // Rh_LaneMarkHeadAngle
                  msg_lane.data[1].c = -value;
                break;
              }
            }
          break;
        }

      }else if(matched_flag_obj == true){ // Object
        char temp_msg_name[30];
        sprintf(buff, "%d", msg_idx_obj[1]+1);

        switch(msg_idx_obj[0]){ // msg_idx_obj[0] indicates whether it's "Obstacle_Data_A_%d" or "Obstacle_Data_B_%d" or "Obstacle_Data_C_%d"
                                // msg_idx_obj[1] indicates number of object sequences, which means "Obstacle_Data_%c_1,2,3,4,...,8"
          case(0): // ObjectIdentifier_A_%d

            for(int i=0; i!=get<1>(msg_list_obj[msg_idx_obj[0]]).size(); i++){
              strcpy(temp_msg_name, get<1>(msg_list_obj[msg_idx_obj[0]])[i]);      
              strcat(temp_msg_name, buff);
              kvaDbGetSignalByName(mh, temp_msg_name, &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // ObjectIdentifier_A_%d
                  temp_msg_obj[msg_idx_obj[1]].id = (int)value;
                  
                  if(msg_idx_obj[1]==0){
                    msg_obj.time = ros::Time::now();
                  }
                break;
              }
            }
          break;

          case(1): // ObjectIdentifier_B_%d

            for(int i=0; i!=get<1>(msg_list_obj[msg_idx_obj[0]]).size(); i++){
              strcpy(temp_msg_name, get<1>(msg_list_obj[msg_idx_obj[0]])[i]);      
              strcat(temp_msg_name, buff);
              kvaDbGetSignalByName(mh, temp_msg_name, &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // Range_B_%d
                  temp_msg_obj[msg_idx_obj[1]].x = value;
                break;

                case(1): // ObjectValidity_B_%d

                  if((int)value == 1){
                    validity_flag[msg_idx_obj[1]] = true; // true : Valid

                  }else{
                    validity_flag[msg_idx_obj[1]] = false; // false : Invalid
                  }
                break;

                case(2): // RangeRate_B_%d
                  temp_msg_obj[msg_idx_obj[1]].vx = value;
                break;

                case(3): // ObjectPostionY_B_%d
                  temp_msg_obj[msg_idx_obj[1]].y = value;
                break;
              }
            }
          break;

          case(2): // ObjectIdentifier_C_%d

            for(int i=0; i!=get<1>(msg_list_obj[msg_idx_obj[0]]).size(); i++){
              strcpy(temp_msg_name, get<1>(msg_list_obj[msg_idx_obj[0]])[i]);      
              strcat(temp_msg_name, buff);
              kvaDbGetSignalByName(mh, temp_msg_name, &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // ObjectType_C_%d

                  switch((int)value){

                    case(0):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Unknown";
                    break;

                    case(1):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Car";
                    break;

                    case(2):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Truck";
                    break;

                    case(3):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Motorcycle";
                    break;

                    case(4):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Other";
                    break;

                    case(5):
                      temp_msg_obj[msg_idx_obj[1]].valid_level = "Pedestrian";
                    break;

                    // default:
                    //   temp_msg_obj[msg_idx_obj[1]].valid_level = "etc";
                    // break;
                  }
                  
                  if(validity_flag[msg_idx_obj[1]] == false){
                    temp_msg_obj[msg_idx_obj[1]].valid_level = "Invalid";
                  }

                  if(msg_idx_obj[1] == 7){ // it indicates that message is "Obstacle_Data_C_8", which means it's time to publish topics
                                           // the normal sequence of incoming object messages are "A_1,A_2,...,A_8,B_1,B_2,...,B_8,C_1,C_2,...,C_8"

                    for(int j=0; j!=obj_num; j++){
                      msg_obj.data.push_back(temp_msg_obj[j]);
                    }

                    pub2.publish(msg_obj);
                    msg_obj.data.clear();

                    for(int j=0; j!=obj_num; j++){
                      temp_msg_obj[j] = {};
                      validity_flag[j] = true;
                    }
                  }
                break;
              }
            }
          break;
        }
      }
    }
    rate.sleep();
  }
}


void CHASSIS_CAN_READER(){
  vector<tuple<char*, vector<char*>>> msg_list;

  msg_list.push_back(make_tuple((char*)"DiagnosticADCU", vector<char*> {(char*)"ADCU_AliveCount",\
                                                                  (char*)"ADCU_SWC_StatCode"}));

  msg_list.push_back(make_tuple((char*)"VCU1_ADCU", vector<char*> {(char*)"vcu_EPS_En",\
                                                                   (char*)"vcu_BRK_En",\
                                                                   (char*)"vcu_ACC_En",\
                                                                   (char*)"vcu_EPS_Override",\
                                                                   (char*)"vcu_BRK_Override",\
                                                                   (char*)"vcu_ACC_Override",\
                                                                   (char*)"vcu_EPS_Status",\
                                                                   (char*)"vcu_ACC_Status",\
                                                                   (char*)"vcu_BRK_Status",\
                                                                   (char*)"vcu_TargetSteerAngle",\
                                                                   (char*)"vcu_MaxSteeringSpeed",\
                                                                   (char*)"vcu_TargetAcceleration",\
                                                                   (char*)"VCU1_ADCU_AliveCnt"}));

  msg_list.push_back(make_tuple((char*)"VCU2_ADCU", vector<char*> {(char*)"vcu_ADMDStatus",\
                                                                   (char*)"vcu_ADReady",\
                                                                   (char*)"vcu_Override",\
                                                                   (char*)"vcu_Estop",\
                                                                   (char*)"vcu_HazardCtl",\
                                                                   (char*)"vcu_LeftTurnCtl",\
                                                                   (char*)"vcu_RightTurnCtl",\
                                                                   (char*)"vcu_LeftTurnState",\
                                                                   (char*)"vcu_RightTurnState",\
                                                                   (char*)"vcu_VB",\
                                                                   (char*)"VCU2_ADCU_AliveCnt"}));

  msg_list.push_back(make_tuple((char*)"VCU3_ADCU", vector<char*> {(char*)"vcu_APS",\
                                                                   (char*)"vcu_BPS",\
                                                                   (char*)"vcu_VS",\
                                                                   (char*)"vcu_SAS_Angle",\
                                                                   (char*)"vcu_SAS_Speed",\
                                                                   (char*)"vcu_LONG_ACCEL"}));                                                      

  int msg_num = 4;
  int temp_substring;
  bool matched_flag = false;
  int msg_idx;
  
  katech_diagnostic_msgs::k_adcu_diagnostic_msg adcu_msg;
  mmc_msgs::chassis_msg msg;

  ros::Rate rate(freq_for_channel_0);

  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);
    
    if(kvaDb_status == kvaDbOK){
      // 먼저 메시지 이름을 가져옴
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      
      // 매칭되는 메시지 찾기
      bool matched_flag = false;
      int msg_idx = -1;
      
      for(int i=0; i < msg_list.size(); i++){
        if(strcmp(buff, get<0>(msg_list[i])) == 0){
          matched_flag = true;
          msg_idx = i;
          break; // 찾았으면 루프 종료
        }
      }
    
    // for(int i=0; i!=msg_list.size(); i++){
        
    //   if(strcmp(buff, get<0>(msg_list[i])) == 0){
    //     matched_flag = true;
    //     msg_idx = i;
    //   }
    

    //   if(matched_flag == false){
    //     msg_idx = 0;
    //   }
    
      // if(kvaDb_status == kvaDbOK){
        // kvaDbGetMsgName(mh, buff, sizeof(buff));
      if(matched_flag){

        switch(msg_idx){

          case(0): // DiagnosticADCU
            msg.time = ros::Time::now();

            for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // ADCU_AliveCount
                  adcu_msg.ADCU_AliveCount++;
                break;

                case(1): // ADCU_SWC_StatCode
                  adcu_msg.ADCU_SWC_StatCode = value; // PI/180 = 0.0174533, rad
                break;
              }
            }
            pub0.publish(adcu_msg);
          break;

          case(1): // VCU1_ADCU

            for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){
                case(0):
                case(1):
                case(2):
                case(3):
                case(4):
                case(5):
                break;

                case(6): // vcu_EPS_Status
                  msg.vcu_EPS_Status = value;
                break;

                case(7): // vcu_ACC_Status
                  msg.vcu_ACC_Status = value;
                break;

                case(8): // vcu_BRK_Status
                  msg.vcu_BRK_Status = value;
                break;

                case(9):
                case(10):
                case(11):
                break;
                
                case(12): // VCU1_ADCU_AliveCnt
                  msg.VCU1_ADCU_AliveCnt = value;
                break;
              }
            }
            pub1.publish(msg);
          break;

          case(2): // VCU2_ADCU

            for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(0): // vcu_ADMDStatus
                  msg.vcu_ADMDStatus = value;
                break;

                case(1): // vcu_ADReady
                  msg.vcu_ADReady = value;
                break;

                case(2): // vcu_Override
                  msg.vcu_Override = value;
                break;

                case(3): // vcu_Estop
                  msg.vcu_Estop = value;
                break;

                case(4): // vcu_HazardCtl
                  msg.vcu_HazardCtl = value;
                break;

                case(5): // vcu_LeftTurnCtl
                  msg.vcu_LeftTurnCtl = value;
                break;

                case(6): // vcu_RightTurnCtl
                  msg.vcu_RightTurnCtl = value; 
                break;

                case(7): // vcu_LeftTurnState
                  msg.vcu_LeftTurnState = value; 
                break;

                case(8): // vcu_RightTurnState
                  msg.vcu_RightTurnState = value; 
                break;

                case(10): // VCU2_ADCU_AliveCnt
                  msg.VCU2_ADCU_AliveCnt = value; 
                break;
              }
            }
          
          break;

          case(3): // VCU3_ADCU

            for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case(2): // vcu_VS
                  msg.vcu_VS = value;
                break;

                case(3): // vcu_SAS_Angle
                  msg.vcu_SAS_Angle = value; 
                break;

                case(5): // vcu_LONG_ACCEL
                  msg.vcu_LONG_ACCEL = value; 
                break;
              }
            }
          break;
        }
      }
    }
    rate.sleep();
  }
}


void PLANNED_PATH_CAN_READER(){
  vector<tuple<char*, vector<char*>>> msg_list;

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_X01",   vector<char*> {(char*)"EGO_TRAJ_X_6",\
                                                                             (char*)"EGO_TRAJ_X_5",\
                                                                             (char*)"EGO_TRAJ_X_4",\
                                                                             (char*)"EGO_TRAJ_X_3",\
                                                                             (char*)"EGO_TRAJ_X_2",\
                                                                             (char*)"EGO_TRAJ_X_1"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_X02",   vector<char*> {(char*)"EGO_TRAJ_X_9",\
                                                                             (char*)"EGO_TRAJ_X_8",\
                                                                             (char*)"EGO_TRAJ_X_7",\
                                                                             (char*)"EGO_TRAJ_X_12",\
                                                                             (char*)"EGO_TRAJ_X_11",\
                                                                             (char*)"EGO_TRAJ_X_10"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_X03",   vector<char*> {(char*)"EGO_TRAJ_X_18",\
                                                                             (char*)"EGO_TRAJ_X_17",\
                                                                             (char*)"EGO_TRAJ_X_16",\
                                                                             (char*)"EGO_TRAJ_X_15",\
                                                                             (char*)"EGO_TRAJ_X_14",\
                                                                             (char*)"EGO_TRAJ_X_13"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_X04",   vector<char*> {(char*)"EGO_TRAJ_X_20",\
                                                                             (char*)"EGO_TRAJ_X_19"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_Y01",   vector<char*> {(char*)"EGO_TRAJ_Y_7",\
                                                                             (char*)"EGO_TRAJ_Y_6",\
                                                                             (char*)"EGO_TRAJ_Y_5",\
                                                                             (char*)"EGO_TRAJ_Y_4",\
                                                                             (char*)"EGO_TRAJ_Y_3",\
                                                                             (char*)"EGO_TRAJ_Y_2",\
                                                                             (char*)"EGO_TRAJ_Y_1"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_Y02",   vector<char*> {(char*)"EGO_TRAJ_Y_9",\
                                                                             (char*)"EGO_TRAJ_Y_8",\
                                                                             (char*)"EGO_TRAJ_Y_14",\
                                                                             (char*)"EGO_TRAJ_Y_13",\
                                                                             (char*)"EGO_TRAJ_Y_12",\
                                                                             (char*)"EGO_TRAJ_Y_11",\
                                                                             (char*)"EGO_TRAJ_Y_10"}));

  msg_list.push_back(make_tuple((char*)"EGO_TRAJ_PRED_Y03",   vector<char*> {(char*)"EGO_TRAJ_Y_20",\
                                                                             (char*)"EGO_TRAJ_Y_19",\
                                                                             (char*)"EGO_TRAJ_Y_18",\
                                                                             (char*)"EGO_TRAJ_Y_17",\
                                                                             (char*)"EGO_TRAJ_Y_16",\
                                                                             (char*)"EGO_TRAJ_Y_15"}));

  char signal_name[20];
  char x_or_y[1];
  int num;
  vector<bool> received_flag_x(4, false);
  vector<bool> received_flag_y(3, false);
  vector<double> points_x(20, 0);
  vector<double> points_y(20, 0);
  
  ros::Rate rate(freq_for_channel_3);

  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_3);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);
    
    if(kvaDb_status == kvaDbOK){
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      sscanf(buff, "EGO_TRAJ_PRED_%c%d", x_or_y, &num);

      if(strcmp(x_or_y, "X") == 0){ // -> if message name = "EGO_TRAJ_PRED_X**"
        received_flag_x[num-1] = true;

      }else{ // -> if message name = "EGO_TRAJ_PRED_Y**"
        received_flag_y[num-1] = true;
      }

      short msg_idx;

      for(short i=0; i!=msg_list.size(); i++){

        if(strcmp(buff, get<0>(msg_list[i]))==0){
          msg_idx = i;
        }
      }

      for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
        kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));
        sscanf(get<1>(msg_list[msg_idx])[i], "EGO_TRAJ_%c_%d", x_or_y, &num);

        if(strcmp(x_or_y, "X") == 0){ // -> if message name = "EGO_TRAJ_PRED_X**"
          points_x[num-1] = value;

        }else{ // -> if message name = "EGO_TRAJ_PRED_Y**"
          points_y[num-1] = value;
        }
      }
      
      // publishing points for visualization
      if(all_of(received_flag_x.begin(), received_flag_x.end(), [](int p){return p==true;}) && \
         all_of(received_flag_y.begin(), received_flag_y.end(), [](int p){return p==true;})){
        visualization_msgs::MarkerArray markerarray;
        visualization_msgs::Marker msg;
        msg.type = msg.POINTS;
        msg.action = msg.ADD;
        msg.id = 0;
        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now();
        
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.6;
        msg.scale.y = 0.6;
        // msg.scale.z = 0.6;
        msg.color.r = 0.4824;
        msg.color.g = 0.9372;
        msg.color.b = 0.6980;
        msg.color.a = 0.9;
        msg.ns = "planned path";
        
        for(short i=0; i!=points_x.size(); i++){
          geometry_msgs::Point point;
          point.x = points_x[i];
          point.y = points_y[i];
          point.z = 0;
          msg.points.push_back(point);
        }
        
        markerarray.markers.push_back(msg);

        pub1.publish(markerarray);

        points_x.clear();
        points_y.clear();
        received_flag_x.clear();
        received_flag_y.clear();

        for(short i=0; i!=20; i++){
          points_x.push_back(999);
          points_y.push_back(999);
        }
        
        for(short i=0; i!=4; i++){
          received_flag_x.push_back(false);
        }

        for(short i=0; i!=3; i++){
          received_flag_y.push_back(false);
        }
      }
    }
    rate.sleep();
  }
}


void FROM_CONTROL_TEAM_CAN_READER(){
  vector<tuple<char*, vector<char*>>> msg_list;

  msg_list.push_back(make_tuple((char*)"from_control_team", vector<char*> {(char*)"lane_change_info",\
                                                                           (char*)"speed_limit"}));

  int msg_idx = 0;
  mmc_msgs::from_control_team msg;
  
  ros::Rate rate(freq_for_channel_2);

  while(ros::ok()){
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_2); // need to check the timeout value
    
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);
    if(kvaDb_status == kvaDbOK){
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      bool matched_flag = false;
      

      for(int i=0; i!=msg_list.size(); i++){
        
        if(strcmp(buff, get<0>(msg_list[i])) == 0){
          matched_flag = true;
          msg_idx = i;
        }
      }

      if(matched_flag == false){
        msg_idx = 0;
      }

      if(matched_flag == true){
        
        switch(msg_idx){

          case 0: // "from_control_team"
            msg.time = ros::Time::now();

            for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
              kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
              kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

              switch(i){

                case 0: // lane_change_info
                  msg.lane_change_info = value;
                break; 

                case 1: // speed_limit
                  msg.speed_limit = value; // kph
                break;
              }
            }

            pub1.publish(msg);
          break;
        }
      }
    }
    rate.sleep();
  }
}