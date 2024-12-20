import cv2
import pid
import time
import tof
import pytesseract
import pointinfo
import markerinfo
from PIL import Image
from robomaster import robot

line = []
markers = []
x, y, w, h = 0, 0, 0, 0

def on_detect_line(line_info):
    line.clear()
    print('line_type', line_info[0])
    for index, item in enumerate(line_info):
        if index == 0:
            continue
        x_, y_, ceta_, c_ = item
        line.append(pointinfo.PointInfo(x_, y_, ceta_, c_))

def on_detect_marker(marker_info):
    global x, y, w, h
    markers.clear()
    for item in marker_info:
        x, y, w, h, info = item
        markers.append(markerinfo.MarkerInfo(x, y, w, h, info))

def setup(kp = 70, ki = 5, kd = 30):
    global ep_robot, pid_ctrl
    global ep_chassis, ep_arm, ep_camera, ep_vision, ep_gripper, ep_sensor
    pid_ctrl = pid.PID(70, 5, 30)
    ep_robot = robot.Robot()
    ep_robot.initialize('ap')
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_arm.recenter().wait_for_completed()
    ep_gripper = ep_robot.gripper
    ep_gripper.open()
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display = False)
    tof.setup()

def recognize():
    global result
    result = ''
    while not result:
        img = ep_camera.read_cv2_image(strategy = 'newest', timeout = 0.5)
        pil_image = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        text = pytesseract.image_to_string(pil_image, lang = 'eng').lower().split('\n\n')
        for string in ('apple', 'pineapple', 'watermelon', 'banana'):
            if string in text:
                result = string
                break
    print('recognize succeed: {}'.format(result))

def get_camera():
    return ep_camera

def get_result():
    return 'apple'

def trans_speed(x, target):
    if x > target:
        return x / target
    else:
        return -x / target + 2

def seek(pos, speed = 30):
    # if pos > 550:
    #     ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=-speed, w4=speed)
    # else:
    #     ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=speed, w4=-speed)
    s = speed * trans_speed(pos, 550)
    if pos > 550:
        s1, s3 = -s, -s
        s2, s4 = s, s
    else:
        s1, s3 = s, s
        s2, s4 = -s, -s
    distance = tof.dis()
    if distance < 20:
        err = trans_speed(distance, 30)
        s1 -= err
        s2 -= err
        s3 -= err
        s4 -= err
    elif distance > 40:
        err = trans_speed(distance, 30)
        s1 += err
        s2 += err
        s3 += err
        s4 += err
    ep_chassis.drive_wheels(w1=s1, w2=s2, w3=s3, w4=s4)
    pass

def grab():
    pass

def move(target_color = 'blue', base_speed = 20, start_angle = 0):
    if start_angle != 0: #turn left
        ep_chassis.move(x = 0, y = 0, z = start_angle, z_speed = 180).wait_for_completed()
    quit_count = 0
    result_line = ep_vision.sub_detect_info(name = 'line', color = target_color, callback = on_detect_line)
    while quit_count <= 10:
        img = ep_camera.read_cv2_image(strategy = 'newest', timeout = 0.5)
        line_1 = line.copy()
        if line_1:
            quit_count = 0
        else:
            quit_count += 1

        min_distance = 1.11
        min_err_x = 0.5
        for index, item in enumerate(line_1):
            cv2.circle(img, item.pt, 3, item.color, -1)
            if item.distance < min_distance:
                min_distance = item.distance
                min_err_x = item._x - 0.5

        #cv2.imshow('Line', img)
        cv2.waitKey(1)
        l_speed = 0
        r_speed = 0
        if min_err_x != 0.5:
            pid_ctrl.set_err(min_err_x)
            dif_speed = pid_ctrl.output
            l_speed = base_speed + dif_speed
            r_speed = base_speed - dif_speed
        ep_chassis.drive_wheels(w2 = l_speed, w3 = l_speed, w1 = r_speed, w4 = r_speed)
        time.sleep(0.1)
    ep_chassis.drive_wheels(0, 0, 0, 0)
    result_line = ep_vision.unsub_detect_info(name = 'line')
    cv2.destroyAllWindows()
    print("move finished")

def place():
    pass

def end():
    ep_camera.stop_video_stream()
    tof.end()
    ep_robot.close()