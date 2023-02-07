from pathlib import PosixPath
import sys
import cv2
import math
import time
import queue
import datetime
import random
import traceback
import threading
import numpy as np
import pyrealsense2 as rs
from xarm import version
from xarm.wrapper import XArmAPI
Home_Pos = [277.7, -38.7, 203.2, 179.8, 1.4, 82.1]
END_Pos = [318.7, 51.2, 167.7, -179.5, -0.6, 78.1]
global POSX , POSY
global dist_to_center
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
# Start streaming
pipeline.start(config)

#定义形状检测函数
class RobotMain(object): 
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._variables = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self):
        try:
            # while self.is_alive:
            #     code = self._arm.set_position(*[277.7, -38.7, 227.7, 179.8, 1.8, 82.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            #     if not self._check_code(code, 'set_position'):
            #         return
            #     code = self._arm.set_position(*[350.0, -38.7, 227.7, 179.8, 1.8, 82.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            #     if not self._check_code(code, 'set_position'):
                    return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_wSarn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)
def ShapeDetection(img,images1,depth_frame):
    global POSX , POSY
    global dist_to_center
    contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)  #寻找轮廓点
    for obj in contours:
        area = cv2.contourArea(obj)  #计算轮廓内区域的面积
        cv2.drawContours(images1, obj, -1, (255, 0, 0), 4)  #绘制轮廓线
        perimeter = cv2.arcLength(obj,True)  #计算轮廓周长
        approx = cv2.approxPolyDP(obj,0.02*perimeter,True)  #获取轮廓角点坐标
        CornerNum = len(approx)   #轮廓角点的数量
        x, y, w, h = cv2.boundingRect(approx)  #获取坐标值和宽度、高度

        #轮廓对象分类
        if CornerNum ==3: objType ="triangle"
        elif CornerNum == 4:
            if w==h: objType= "Square"
            else:objType="Rectangle"
        elif CornerNum>4: objType= "Circle"
        else:objType="N"

        if (x > 80 and x < (640 - 80)) and (w >50 and h > 50):
            cv2.rectangle(images1,(x,y),(x+w,y+h),(0,0,255),2)  #绘制边界框
            POSX = x+w//2
            POSY = y+h//2
            #print(POSX)
            cv2.circle(images1, (x+(w//2),y+(h//2)), 1, (0, 0, 255), 0)
            #cv2.putText(images1,objType,(x+(w//2),y+(h//2)),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,0),1)  #绘制文字
            dist_to_center = depth_frame.get_distance(POSX - 70, POSY)
            dist_to_center = str(round(dist_to_center * 100, 5))
            print("D:" + dist_to_center + "Pos:"+ str(x+(w//2)) + "," + str(y+(h//2)))
            cv2.putText(images1,"D:"+ dist_to_center + "cm",(15,30),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,255),1)  #绘制文字
            cv2.putText(images1,"Pos:"+ str(x+(w//2)) + "," + str(y+(h//2)) ,(15,60),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,255),1)  #绘制文字
            dist_to_center = float(dist_to_center)

            

def Camera():
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()
    ir_frame_left = frames.get_infrared_frame(1)
    ir_frame_right = frames.get_infrared_frame(2)
    ir_left_image = np.asanyarray(ir_frame_left.get_data())
    ir_right_image = np.asanyarray(ir_frame_right.get_data())
    images1 = np.hstack((ir_right_image, ir_right_image))

    imgBlur = cv2.GaussianBlur(ir_right_image,(5,5),1)
    imgCanny = cv2.Canny(imgBlur,270,270)
    ShapeDetection(imgCanny,images1,depth_frame)  #形状检测

    images1 = cv2.rectangle(images1, (80, 0), (640 -80, 480), (0, 0, 255), 2)
    images1 = np.hstack((images1, imgCanny)) #视觉坐标X- = Y+  X- = Y-
    #cv2.imshow('RealSense', images1)

def Xarm_RP(arm,x,y,z,SYNC):
    Home_Pos[0] = Home_Pos[0] + x
    Home_Pos[1] = Home_Pos[1] + y
    Home_Pos[2] = Home_Pos[2] + z
    arm.set_position(*Home_Pos, speed=50, wait=SYNC)

def Xarm_run(arm):
    for i in range(5):
        Camera() 
        Bf_Cx = POSX
        Xarm_RP(arm,0,1,0,True)
        Camera() 
        Di = POSX - Bf_Cx
        print("DI:" + str(Di))
        Xarm_RP(arm,-((POSY - 240)/Di),-((POSX - 320)/Di),0,True)
    Xarm_RP(arm,58.2,-42.4,0,True)
    Xarm_RP(arm,0,0,90 - (dist_to_center*10),True)
    #arm._arm.set_cgpio_digital(7, 1, delay_sec=0)
    arm.set_cgpio_digital(0, 1)
    Xarm_RP(arm,0,0,80,True)
    arm.set_position(*END_Pos, speed=50, wait=True)
    arm.set_cgpio_digital(0, 0)
    print(str(90 - (dist_to_center*10)))


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.231', baud_checkset=False)
    robot_main = RobotMain(arm)
    arm.set_cgpio_digital(0, 0)
    arm.set_position(*Home_Pos, speed=50, wait=True)
    print(arm.get_position(), arm.get_position(is_radian=True))
    #robot_main.run()
    Xarm_run(arm)

