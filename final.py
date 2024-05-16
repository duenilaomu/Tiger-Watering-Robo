# encoding: utf-8
from robomaster import robot,camera
import cv2
import numpy as np
import time
import serial


def move_left():
    # distance = 0.1
    # rm.chassis.move(x=0, y=-distance, z=0, xy_speed=speed).wait_for_completed()
    rm.chassis.drive_speed(x=0, y=-speed, z=0, timeout=5)
    # time.sleep(0.1)

# 控制机械臂伸出
def extend_arm():
    rm.robotic_arm.move(x=40, y=-40).wait_for_completed()
# 控制机械臂收回
def retract_arm():
    rm.robotic_arm.move(x=-40, y=40).wait_for_completed()
# 检测黄色
def detect_yellow(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([26, 100, 100])
    upper_yellow = np.array([34, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return mask

def sub_data_handler(sub_info):
    pos_x, pos_y = sub_info
    print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))

def crop_center(image, crop_width, crop_height):
    height, width = image.shape[:2]
    start_x = width // 2 - crop_width // 2
    start_y = height // 2 - crop_height // 2
    return image[start_y:start_y + crop_height, start_x:start_x + crop_width]


if __name__ == '__main__':
    Port = "/dev/ttyACM0"
    baudRate = 9600
    ser=serial.Serial(Port,baudRate,timeout=1)
    time.sleep(20)
    # 连接RoboMaster
    rm = robot.Robot()
    # 启动连接
    rm.initialize(conn_type="ap")
    # 获取摄像头
    ep_camera = rm.camera
    # 设置速度
    speed = 0.2

    # rm.robotic_arm.sub_position(freq=5, callback=sub_data_handler)
    rm.robotic_arm.moveto(80, 80)
    rm.gripper.close()
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    # 假设截取尺寸
    crop_width, crop_height = 100, 240
    # 主循环
    while True:
        # 获取视频流帧
        frame = ep_camera.read_cv2_image(timeout=1, strategy="newest")
        # cv2.waitKey(1)
        # time.sleep(1)
        # 缩小图像
        resized_frame = cv2.resize(frame, (320, 240))
        # 截取中间区域
        cropped_frame = crop_center(resized_frame, crop_width, crop_height)
        # 检测黄色区域
        yellow_mask = detect_yellow(cropped_frame)
        # 查找黄色区域
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 显示图像
        cv2.imshow("Frame", cropped_frame)
        # 如果识别到黄色区域
        if contours:
            # # 停止移动
            rm.chassis.stop()
            # 伸出机械臂
            extend_arm()
            # water flower
            send="1"
            ser.write(send.encode())
            ser.close()
            ser=serial.Serial(Port,baudRate,timeout=1)
            # 等待3秒
            time.sleep(3)
            # 收回机械臂
            retract_arm()
            # 继续向左移动
            move_left()
            ep_camera.stop_video_stream()
            ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
            time.sleep(3)
        else:
            # 继续向左移动
            move_left()

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    ser.close()
    rm.chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    rm.robotic_arm.unsub_position()
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    # 关闭连接
    rm.close()

