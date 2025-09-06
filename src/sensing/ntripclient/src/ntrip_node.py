#! /usr/bin/python3
# print("start")
## Get current lat, lon
import serial
import rospy
import configparser
from pyproj import Transformer
import os
import subprocess
import signal
from threading import Thread
import time
import csv
from mmc_msgs.msg import ntrip_status


# Parameter
station_distance_threshold = 30.0 # [km]
DIRECTORY = rospy.get_param("ntrip_directory")
f = open(DIRECTORY +'/data/gnss.csv','r')
rdr = csv.reader(f)
rdr_ = []
for line in rdr:
    rdr_.append(line)

def connect_ntrip_client():
    global process_
    global initialized
    initialized = False
    while not rospy.is_shutdown():
        if (not initialized):
            continue
        # print("Connection start")
        process_ = subprocess.Popen( [DIRECTORY + '/src/connect_ntrip_from_base.sh', DIRECTORY ], universal_newlines=True, shell=False, stdout=subprocess.PIPE, start_new_session=True)
        process_.communicate()
        # if (initialized):
        #     print("disconnected ntrip, connect again")
        # else:
        #     print("disconnected ntrip.")

def find_my_location():
    c = configparser.ConfigParser()
    c.read( DIRECTORY + '/src/config.ini')
    dev_gps = c['DEFAULT']['DEV_GPS']
    # init_lat = 37.557
    # init_lon = 127.04577409865647 
    # with serial.Serial(dev_gps, 115200, timeout = 5) as ser:
    with serial.Serial('/dev/ttyUSB2', 115200, timeout = 5) as ser:
    # with serial.Serial('/dev/ttyUSB5', 115200, timeout = 5) as ser:
        # print('serial')
        # for i in range(50):
        while True:
            line = ser.readline()
            line_ = str(line)
            line_parsed = line_.split(',')
            # print(line)

            if 'GGA' in line_ :
                try:
                    lat_str = line_parsed[2]
                    lon_str = line_parsed[4]

                    lat_str_deg = float(lat_str[0:2])
                    lon_str_deg = float(lon_str[0:3])

                    lat_str_min = float(lat_str[2:])
                    lon_str_min = float(lon_str[3:])

                    init_lat = lat_str_deg + lat_str_min/60.0
                    init_lon = lon_str_deg + lon_str_min/60.0

                    # print('lat: ', init_lat, 'lon:', init_lon)

                    break
                except:
                    init_lat = -1000
                    init_lon = -1000
    return init_lat, init_lon

def get_station_from_my_loc(init_lat, init_lon):
    global initialized
    mount_point_ = 'none'
    lat_ = -1.0
    lon_ = -1.0
    rtk_status = 0
    if (not os.path.exists( DIRECTORY +'/data/mountpoint')):
        # print('getting ntrip server lists ........')
        run_get_ntrip_lists = ['']
        while (run_get_ntrip_lists == ['']):
            run_get_ntrip_lists = subprocess.check_output([DIRECTORY + '/src/get_ntrip_lists.sh', DIRECTORY + '/src'],shell=False , universal_newlines=True).split('\n')
    else:
        from math import sqrt
        tftf = Transformer.from_crs("epsg:4326", "epsg:5179")
        init_north, init_east = tftf.transform(init_lat, init_lon)
        with open( DIRECTORY + '/data/nearest_mount_lat','r') as f:
            lat_ = float(f.read())
        with open( DIRECTORY + '/data/nearest_mount_lon','r') as f:
            lon_ = float(f.read())
        with open( DIRECTORY + '/data/mountpoint','r') as f:
            mount_point_ = f.read()
        north, east= tftf.transform(lat_, lon_)
        dnorth = north - init_north
        deast = east - init_east
        dist = sqrt(dnorth**2 + deast**2)/1000
        if (dist < station_distance_threshold):
            initialized = True
            # print("station distance is ", dist, "km, we will DO NOT CHANGE base station")
            rtk_msg_tuple = (mount_point_, 
                            mount_point_, 
                            lat_,
                            lon_,
                            1,
                            dist,
                            dist)
            return rtk_msg_tuple
        else:
            # print("station distance is ", dist, "km, we will SEARCH base station")
            run_get_ntrip_lists = ['']
            while (run_get_ntrip_lists == ['']):
                run_get_ntrip_lists = subprocess.check_output([DIRECTORY + '/src/get_ntrip_lists.sh', DIRECTORY + '/src'],shell=False , universal_newlines=True).split('\n')
        

    mount_ = []
    lat_list = []
    lon_list = []

    for line in run_get_ntrip_lists:
        stream_info = line.split(';')

        if len(stream_info) < 10:
            continue

        if 'RTCM 3' not in stream_info[3]:
        #if 'RTCM 2' not in stream_info[3]:
            continue

        mount_point, lat, lon, rtcm_ver = stream_info[1], stream_info[9], stream_info[10], stream_info[3]

        if not lat:
            # latitude info is empty
            continue
        lat_acc = lat
        lon_acc = lon
        for line_ in rdr_:
            if (mount_point.split('-')[0] in line_):
                lat_acc = float(line_[1])
                lon_acc = float(line_[2])
        mount_.append(mount_point)
        lat_list.append(lat_acc)
        lon_list.append(lon_acc)


    # ## Get nearest mount point
    from math import sqrt
    tftf = Transformer.from_crs("epsg:4326", "epsg:5179")
    init_north, init_east = tftf.transform(init_lat, init_lon)

    min_dist = 100
    for mount, lat, lon in zip(mount_, lat_list, lon_list):
        north, east= tftf.transform(float(lat), float(lon))
        dnorth = north - init_north
        deast = east - init_east

        dist = sqrt(dnorth**2 + deast**2)/1000

        if dist < min_dist:
            min_dist = dist
            nearest_mount = mount
            nearest_mount_lat = float(lat)
            nearest_mount_lon = float(lon)


    # print('nearest mount:', nearest_mount)
    # print('minimum distance:', min_dist, "km")        

    
    with open(DIRECTORY+ '/data/nearest_mount_lat','w') as f:
        f.write(str(nearest_mount_lat))
        
    with open(DIRECTORY+'/data/nearest_mount_lon','w') as f:
        f.write(str(nearest_mount_lon))

    with open(DIRECTORY+'/data/mountpoint','w') as f:
        f.write(nearest_mount)

    
    if (initialized):
        if (process_.poll() is None):
            os.killpg(process_.pid, signal.SIGTERM)
        if (min_dist > station_distance_threshold):
            # print("station distance is ", min_dist, "km, need to be reset base station")
            initialized = False
            if (process_.poll() is None):
                os.killpg(process_.pid, signal.SIGTERM)
            mount_point_ = 'none'
            rtk_msg_tuple = (mount_point_, 
                            nearest_mount, 
                            -1,
                            -1,
                            0,
                            -1,
                            min_dist)
            return rtk_msg_tuple
    if initialized:
        rtk_msg_tuple = (mount_point_, 
                        nearest_mount, 
                        lat_,
                        lon_,
                        1,
                        dist,
                        min_dist)
    else:
        rtk_msg_tuple = ("none", 
                        nearest_mount, 
                        -1,
                        -1,
                        0,
                        -1,
                        min_dist)
    if (min_dist < station_distance_threshold):
        initialized = True
    return rtk_msg_tuple

def get_station_distance_forever():
    r = rospy.Rate(1)
    ntrip_status_pub = rospy.Publisher('/localization/ntrip', ntrip_status, queue_size=1, latch=True)
    while not rospy.is_shutdown():
        lat, lon = find_my_location()
        if lat < -500:
            # print('cannot find my current location, check gps data')
            continue        
        rtk_msg_tuple = get_station_from_my_loc(lat, lon)
        ntrip_status_msg = ntrip_status()
        ntrip_status_msg.header.stamp = rospy.Time.now()
        ntrip_status_msg.current_mount_point = rtk_msg_tuple[0]
        ntrip_status_msg.nearest_mount_point = rtk_msg_tuple[1]
        ntrip_status_msg.current_mount_point_latitude = rtk_msg_tuple[2]
        ntrip_status_msg.current_mount_point_longitude = rtk_msg_tuple[3]
        ntrip_status_msg.rtk_status = rtk_msg_tuple[4]
        ntrip_status_msg.current_station_distance = rtk_msg_tuple[5]
        ntrip_status_msg.nearest_station_distance = rtk_msg_tuple[6]
        ntrip_status_pub.publish(ntrip_status_msg)
        r.sleep()
    if (process_.poll() is None):
        os.killpg(process_.pid, signal.SIGTERM)


if __name__=='__main__':
    
    rospy.init_node('ntrip_client')
    t1 = Thread(target=connect_ntrip_client)
    t2 = Thread(target=get_station_distance_forever)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

        
        
    