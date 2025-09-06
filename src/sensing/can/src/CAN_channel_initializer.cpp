#include <can_pub.h>


int main(int argc, char **argv){
  
  cout<<"CAN Channel Initializing ..."<<endl;
  ros::init(argc, argv, "CAN_channel_initializer");
  ros::NodeHandle node("~");

  int channel_num = 4;
  int channel_list[channel_num] = {0,1,2,3}; // channel 0: chassis, camera, front radar
                                             // channel 1: front lidar(Ibeo LUX)
                                             // channel 2: P-CAN(Perception output + Localization output)
                                             // channel 3: HMI CAN

  canInitializeLibrary();

  for(int i=0; i!=channel_num; i++){
    // hCAN = canOpenChannel(channel_list[i], canOPEN_EXCLUSIVE);
    hCAN = canOpenChannel(channel_list[i], canOPEN_REQUIRE_INIT_ACCESS);
    can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
    can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
    can_status = canBusOn(hCAN);

    if(hCAN==canOK){
      cout<<"The CAN channel "<<channel_list[i]<<" has been opened successfully..."<<endl;
    }
    
    canBusOff(hCAN);
    canClose(hCAN);
  }
}