################################################## BSMcoreData ##################################################
uint8 MsgCount      # 0 ~ 127
uint32 Temporary_ID # -
uint16 DSecond      # Unit : msec

float64 GPS_Position_Latitude  # Unit : 10^-7 deg ## 실제로는 Tucson의 global east * 0.0001 다
float64 GPS_Position_Longitude # Unit : 10^-7 deg ## 실제로는 Tucson의 global north * 0.0001 다
float64 GPS_Position_Elevation # Unit : 0.1 m

float64 Position_Accuracy_SemiMajorAxisAccuracy    # Unit : 0.05 m ## 실제로는 Tucson의 /sensors/gps/fix 의 position_covariance의 0번째 entry 다
float64 Position_Accuracy_SemiMinorAxisAccuracy    # Unit : 0.05 m ## 실제로는 Tucson의 /sensors/gps/fix 의 position_covariance의 4번째 entry 다
float64 Position_Accuracy_SemiMajorAxisOrientation # Unit : 360/35535 deg (0.0101 deg) 


uint8 TransmissionState # Possible cases below
                         #       "Neutral" : 0    -> 0x00
                         #          "Park" : 1     -> 0x01
                         # "Forward_Gears" : 2 -> 0x02
                         # "Reverse_Gears" : 3 -> 0x03
                         #   "Unavailable"  : 7  -> 0x07
 

float64 Speed              # Unit : 0.02 m/s   ## 실제로는 Tucson의 chassis speed 다
float64 Heading            # Unit : 0.0125 deg ## 실제로는 Tucson의 global yaw 다
float64 SteeringWheelAngle # Unit : 1.5 deg

float64 Acceleration_Set_4Way_Longitude_Acceleration    # Unit : 0.01 m/s^2
float64 Acceleration_Set_4Way_Latitude_Acceleration     # Unit : 0.01 m/s^2
float64 Acceleration_Set_4Way_Vertical_Acceleration = 0 # Unit : 0.02 G, Value : only 0 (no DB)
float64 Acceleration_Set_4Way_YawRate                   # Unit : 0.01 deg/sec

uint8 BrakeSystemStatus_Brake_Applied_Status # Value : 0 or 1
                                             # 0 -> B'00000
                                             # 1 -> B'11110

string BrakeSystemStatus_Traction_Control_Status = "Unavailable"
                                                   # Value : only "Unavailable" (no DB) -> B'00


string BrakeSystemStatus_Anti_Lock_Brake_Status # Possible cases below
                                                #          "On" : B'10
                                                #     "Engaged" : B'11


string BrakeSystemStatus_Stability_Control_Status = "Unavailable"
                                                   # Value : only "Unavailable" (no DB) -> B'00


uint8 BrakeSystemStatus_Brake_Boost_Applied=0    # Value : only 0 (no DB) # 실제로는 TUCSON의 우측깜빡이 시그널이다
uint8 BrakeSystemStatus_Auxiliary_Brake_Status=0 # Value : only 0 (no DB) # 실제로는 TUCSON의 좌측깜빡이 시그널이다

float32 Vehicle_Size_VehicleWidth  = 175 # Unit : 1 cm
float32 Vehicle_Size_VehicleLength = 425 # Unit : 1 cm



################################################## Part II Content ##################################################
############################### Vehicle Safety Extensions ###############################
uint8 Vehicle_Event_Flag_eventHazardLights = 0              # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventStopLineViolation = 0         # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventABSactivated                  # Value : 0 or 1 (0:Inactive, 1:active)
uint8 Vehicle_Event_Flag_eventTractionControlLoss = 1       # Value : only 1 (no DB)
uint8 Vehicle_Event_Flag_eventStabilityControlactivated = 0 # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventHazardousMaterials = 0        # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventReserved1 = 0                 # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventHardBraking = 0               # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventLightsChanged = 0             # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventWipersChanged = 0             # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventFlatTire = 0                  # Value : only 0 (no DB)
uint8 Vehicle_Event_Flag_eventDisabledVehicle               # 
uint8 Vehicle_Event_Flag_eventAirBagDeployment = 0          # Value : only 0 (no DB)
                                                            # example : eventABSactivated = 1 and eventTractionControlLoss=1 ->  B'0000000001100

uint8 Sequence_Size_of_Path_History_Point = 10 # 0 ~ 10

path_history[] Path_History_Point     # Include : Lat, Longi, Elev, Time, Speed information

uint8 Flag_of_Exterior_Lights = 0 # Value : only 0 (no DB)
uint8 Exterior_Lights = 0         # Value : only 0 (no DB)


############################### Special Vehicle Extensions ###############################
uint8 Emergency_Detail_SSP_index = 0        # Value : only 0 (no DB)
uint8 Emergency_Detail_Siren_In_Use = 0     # Value : only 0 (no DB)
uint8 Emergency_Detail_Light_bar_In_Use = 0 # Value : only 0 (no DB)

uint8 Flag_of_Event_Description = 0     # Value : only 0 (no DB)
uint8 Event_Description_Type_Event = 0  # Value : only 0 (no DB)
uint8 Event_Description_Description = 0 # Value : only 0 (no DB)


############################### Supplemental Vehicle Extensions ###############################
uint8 Vehicle_Classification_Basic_Vehicle_Class = 10 # Value : only 10 (default type = 10)
uint8 Vehicle_Classification_Fuel_Type = 3            # Value : only 3 (diesel = 3)