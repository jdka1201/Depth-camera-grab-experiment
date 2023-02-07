import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

# Start streaming
pipeline.start(config)


#定义形状检测函数
def ShapeDetection(img):
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
            POSX = int((x+w/2)-70)
            POSY = int(y+h/2)
            #print(POSX)
            cv2.circle(images1, (x+(w//2),y+(h//2)), 1, (0, 0, 255), 0)
            #cv2.putText(images1,objType,(x+(w//2),y+(h//2)),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,0),1)  #绘制文字
        
            dist_to_center = depth_frame.get_distance(POSX,y+(h//2))
            dist_to_center = str(round(dist_to_center * 100, 5))
            print(dist_to_center)
            cv2.putText(images1,"D:"+ dist_to_center + "cm",(15,30),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,255),1)  #绘制文字
            cv2.putText(images1,"Pos:"+ str(x+(w//2)) + "," + str(y+(h//2)) ,(15,60),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,0,255),1)  #绘制文字

            

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        frameds = pipeline.wait_for_frames()
        # alignIt = rs.align(rs.stream.depth) # rs.stream.depth
        # frames = alignIt.process(frames)
        depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()
        ir_frame_left = frames.get_infrared_frame(1)
        ir_frame_right = frames.get_infrared_frame(2)
       
        # if not depth_frame or not color_frame:
        #     continue
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
       
        ir_left_image = np.asanyarray(ir_frame_left.get_data())
        ir_right_image = np.asanyarray(ir_frame_right.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Stack both images horizontally
        images2 = np.hstack(( depth_colormap))
        images1 = np.hstack((ir_right_image, ir_right_image))
        cv2.imshow('depth_image', images2)
        #opencv 处理部分
        imgBlur = cv2.GaussianBlur(ir_right_image,(5,5),1)
        imgCanny = cv2.Canny(imgBlur,270,270)
        images1 = cv2.rectangle(images1, (80, 0), (640 -80, 480), (0, 0, 255), 2)
        images1 = np.hstack((images1, imgCanny))
        cv2.circle(images1, (320+70,240), 1, (0, 0, 255), 0)
        cv2.imshow('RealSense', images1)
        
        dist_to_center = depth_frame.get_distance(320, 240)
        print(dist_to_center)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    # Stop streaming
    pipeline.stop()
