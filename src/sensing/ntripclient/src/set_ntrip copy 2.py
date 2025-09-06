#! /usr/bin/python3
print("start")
## Get current lat, lon
# import serial
import configparser
from pyproj import Transformer
import os
import subprocess
import signal
from threading import Thread
import time

# Parameter
station_distance_threshold = 15.0 # [km]

def connect_ntrip_client():
    global process_
    global initialized
    initialized = False
    while(1):
        if (not initialized):
            continue
        try:
            process_ = subprocess.Popen('./connect_ntrip_from_base.sh', universal_newlines=True, shell=True, stdout=subprocess.PIPE, start_new_session=True)
            process_.communicate()
        except KeyboardInterrupt:
            exit()
        print("disconnected ntrip, connect again")

def find_my_location():
    c = configparser.ConfigParser()
    c.read('config.ini')
    dev_gps = c['DEFAULT']['DEV_GPS']
    init_lat = 38.457302735342866
    init_lon = 128.04577409865647 
    # with serial.Serial(dev_gps, 115200, timeout = 5) as ser:
    #     print('serial')
    #     # for i in range(50):
    #     while True:
    #         line = ser.readline()
    #         line_ = str(line)
    #         line_parsed = line_.split(',')
    #         # print(line)

    #         if 'GGA' in line_ :
    #             try:
    #                 lat_str = line_parsed[2]
    #                 lon_str = line_parsed[4]

    #                 lat_str_deg = float(lat_str[0:2])
    #                 lon_str_deg = float(lon_str[0:3])

    #                 lat_str_min = float(lat_str[2:])
    #                 lon_str_min = float(lon_str[3:])

    #                 init_lat = lat_str_deg + lat_str_min/60.0
    #                 init_lon = lon_str_deg + lon_str_min/60.0

    #                 print('lat: ', init_lat, 'lon:', init_lon)

    #                 break
    #             except:
    #                 init_lat = -1000
    #                 init_lon = -1000
    return init_lat, init_lon

def get_station_from_my_loc(init_lat, init_lon):
    global initialized
    if (not os.path.exists('/tmp/mountpoint')):
        print('getting ntrip server lists ........')
        run_get_ntrip_lists = ['']
        while (run_get_ntrip_lists == ['']):
            run_get_ntrip_lists = subprocess.check_output('./get_ntrip_lists.sh', universal_newlines=True).split('\n')
    else:
        from math import sqrt
        tftf = Transformer.from_crs("epsg:4326", "epsg:5179")
        init_north, init_east = tftf.transform(init_lat, init_lon)
        with open('/tmp/nearest_mount_lat','r') as f:
            lat = float(f.read())
        with open('/tmp/nearest_mount_lon','r') as f:
            lon = float(f.read())
        north, east= tftf.transform(lat, lon)
        dnorth = north - init_north
        deast = east - init_east
        dist = sqrt(dnorth**2 + deast**2)/1000
        if (dist < station_distance_threshold):
            initialized = True
            print("station distance is ", dist, "km, we will DO NOT CHANGE base station")
            return dist
        else:
            print("station distance is ", dist, "km, we will SEARCH base station")
            run_get_ntrip_lists = ['']
            while (run_get_ntrip_lists == ['']):
                run_get_ntrip_lists = subprocess.check_output('./get_ntrip_lists.sh', universal_newlines=True).split('\n')
        

    mount_ = []
    lat_ = []
    lon_ = []

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


        mount_.append(mount_point)
        lat_.append(lat)
        lon_.append(lon)


    # ## Get nearest mount point
    from math import sqrt
    tftf = Transformer.from_crs("epsg:4326", "epsg:5179")
    init_north, init_east = tftf.transform(init_lat, init_lon)

    min_dist = 100
    for mount, lat, lon in zip(mount_, lat_, lon_):
        north, east= tftf.transform(float(lat), float(lon))
        dnorth = north - init_north
        deast = east - init_east

        dist = sqrt(dnorth**2 + deast**2)/1000

        if dist < min_dist:
            min_dist = dist
            nearest_mount = mount
            nearest_mount_lat = float(lat)
            nearest_mount_lon = float(lon)


    print('nearest mount:', nearest_mount)
    print('minimum distance:', min_dist, "km")
    if (min_dist > station_distance_threshold):
        print("station distance is ", min_dist, "km, need to be reset base station")
        # return min_dist

    # ## Set options for ntripclient

    # In[9]:
    
    with open('/tmp/nearest_mount_lat','w') as f:
        f.write(str(nearest_mount_lat))
        
    with open('/tmp/nearest_mount_lon','w') as f:
        f.write(str(nearest_mount_lon))

    with open('/tmp/mountpoint','w') as f:
        f.write(nearest_mount)

    initialized = True
    if (initialized):
        os.killpg(process_.pid, signal.SIGTERM)
        with open('/tmp/mountpoint','r') as f:
            if (min_dist > station_distance_threshold):
                initialized = False
                os.killpg(process_.pid, signal.SIGTERM)

    return min_dist

def get_station_distance_forever():
    global station_distance
    while(1):
        lat, lon = find_my_location()
        if lat < -500:
            print('cannot find my current location, check gps data')
            continue        
        station_distance = get_station_from_my_loc(lat, lon)
        time.sleep(120)


if __name__=='__main__':
    try:
        t1 = Thread(target=connect_ntrip_client)
        t2 = Thread(target=get_station_distance_forever)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except KeyboardInterrupt:
        os.killpg(process_.pid, signal.SIGTERM)
        exit()

        
        
    