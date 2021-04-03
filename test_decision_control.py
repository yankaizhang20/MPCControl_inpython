#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import json
import time
import math
# import rospy
# import requests
import numpy as np
import xlrd
# from cloud_message_process.msg import car_info_message
# from create_trajectory import wgs84toWebMercator
# from create_trajectory import imu2rearmiddle
# from create_trajectory import imu2frontmiddle
# from latlontomap import transform


#! 域名访问决策控制的地址
url = "http://vehicleroadcloud.faw.cn:60443/Control/Control/Get/Control"

#! 直接访问决策控制地址（需要挂vpn）
# url = "http://10.112.4.18:30965/Control/Get/Control"

#! app后台的视频请求接口
# url1 = "http://vehicleroadcloud.faw.cn:60443/backend/appBackend/videoRequest"

#获取excel中的路点
def GetRefTrack(path):
    file = xlrd.open_workbook(path)
    sheet = file.sheets()[0]
    rows = sheet.nrows
    cols = sheet.ncols
    array = np.zeros((rows, cols))
    for x in range(cols):
        collist = sheet.col_values(x)
        col = np.matrix(collist)
        array[:, x] = col
    trajectory=[]
    for i in range(rows):
        tra = dict()
        tra["time"] = 0.02 * i
        tra["x_point"] = array[i, 0]
        tra["y_point"] = array[i, 1]
        tra["heading"] = array[i, 2]
        tra["speed"] = array[i, 3]
        tra["angle"] = array[i, 4]
        trajectory.append(tra)
    return trajectory
def test_control_decision():
        requestData = dict()
        requestData["vin"] = "111111"
        requestData["userid"] = "111111"
        requestData["requestid"] = "111111"
        requestData["type"] = 0
        car = dict()
        car["front_wheel_x"] = 100.00
        car["front_wheel_y"] = 100.00
        car["rear_wheel_x"] = 98.00
        car["rear_wheel_y"] = 98.00
        car["speed"] = 5.5
        car["heading"] = 1.5
        car["accelerate"] = 3.0
        car["angle"] = 1.10
        requestData["car"] = car
        trajectory = []
        path = "D:\\trackpath_sim\\light\\roads.xlsx"
        trajectory = GetRefTrack(path)
        # tra = dict()
        # tra["time"] = 0.01
        # tra["x_point"] = 105.204
        # tra["y_point"] = 105.204
        # tra["heading"] = 1.5
        # tra["speed"] = 3.00
        # tra["angle"] = 0.11
        # trajectory.append(tra)
        # tra = dict()
        # tra["time"] = 0.02
        # tra["x_point"] = 106.204
        # tra["y_point"] = 106.204
        # tra["heading"] = 1.5
        # tra["speed"] = 3.00
        # tra["angle"] = 0.11
        # trajectory.append(tra)
        requestData["trajectory"] = trajectory
        ret = requests.post(url, json=trajectory, timeout = 5)
        print(ret)


def callback(data):
        f = open("trajectory_point.json")
        cur_line = f.readline()
        file_json = json.loads(cur_line)
        #循环轨迹点中没有列表
        if len(file_json["dict"])>0:
                count = 0
                while True:
                        cur_json = file_json["dict"][count]
                        
                        #! 最终返回的json串(基础信息)
                        result = {}
                        result["vin"] = "111111"
                        result["userid"] = "111111"
                        t = time.time()
                        result["timestamp"] = round(t * 1000)
                        result["requestid"] = "1111111"
                        result["type"] = 11

                        #! 最终返回的json串（其他信息）
                        # * 车况数据
                        result["car"] = {}
                        result["car"]["speed"] = data.car_speed
                        longtitude = data.longtitude
                        latitude = data.latitude
                        heading = data.heading
                        imu_position_x,imu_position_y = transform(longtitude,latitude)
                        cur_front_wheel_x, cur_front_wheel_y = imu2frontmiddle(imu_position_x, imu_position_y, heading)
                        cur_rear_wheel_x, cur_rear_wheel_y = imu2rearmiddle(imu_position_x, imu_position_y, heading)


                        #TODO 车辆前后轮中心点位置
                        result["car"]["front_wheel_x"] = cur_front_wheel_x
                        result["car"]["front_wheel_y"] = cur_front_wheel_y
                        result["car"]["rear_wheel_x"] = cur_rear_wheel_x
                        result["car"]["rear_wheel_y"] = cur_rear_wheel_y
                        result["car"]["heading"] = heading
                        #纵向加速度
                        longtitudinalaccl = data.longtitudinalaccl
                        #横向加速度
                        lateraaccel = data.lateraaccel
                        #加速度加速为正，减速为负
                        acceleration = math.sqrt(pow(longtitudinalaccl,2) + pow(lateraaccel, 2)) * np.sign(longtitudinalaccl)
                        result["car"]["accelerate"] = acceleration
                        #TODO 车轮转角是否需要转化
                        result["car"]["angle"] = data.wheel_angle

                        # * 轨迹点数据
                        sum = cur_json["time"]
                        result["trajectory"] = cur_json
                        cur_json["time"] = sum-1
                        #发的次数足够了，删除上一次的点
                        if cur_line["dict"][0]["time"] == 0:
                                cur_line["dict"].remove(cur_line["dict"][0])
                        ret = requests.post(url, json=result, timeout = 5)
                        print(ret)
                        text = json.loads(ret.text)
                        print(text)
                        if len(cur_line["dict"][0]["time"] == 0):
                                break

if __name__ == '__main__':
        path="D:\\trackpath_sim\\light\\roads.xlsx"
        trajectory=GetRefTrack(path)
        print(len(trajectory))
        # test_control_decision()
        # rospy.init_node('test_decision_control', anonymous=False)
        # car_info_subscriber = rospy.Subscriber("c  ar_info_to_test_decision_control", car_info_message, callback)
        # rospy.spin()