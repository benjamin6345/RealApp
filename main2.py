from ultralytics import YOLO
import cv2
import cvzone
import time
import math
import numpy as np

model = YOLO('best3.pt')
car_model = YOLO('yolov8n.pt')

VIDEO_PATH = 0 #影片位置

capture = cv2.VideoCapture(VIDEO_PATH)
WIDTH, HEIGHT = int(capture.get(3)), int(capture.get(4))

car_classes = {1,2,3,4,5,6,7,8}
people_class = 0 #人的分类是0

#人与车开始出现在屏幕的时间
global_car_start_time = dict() 
global_people_start_time = dict()

#比例越大，越讓車先通過。比例越小，越讓人先通過


#等待了一会儿的人与车Id
waiting_car_id = set()
waiting_people_id = set()

detected_people_id = set()
detected_car_id = set()


wait_time_bound = 10 #等待十秒为止“正在等待的人与车”
recalc_time_interv = 3 #每隔三秒计算有多少人与车在等

time_interv_started = False
time_interv_start_time = 0


car_green_light_bound = 0.45 #比例达到多少换红灯，紅燈指人燈紅燈
car_red_light_bound= 0.29
car_people_ratio = 0

calc_time = 0

changed_light_time = time.time()
shortest_dur = 15 #红灯最长维持时间，单位秒
longest_dur = 45 #红灯最短维持时间，单位秒

# green_image_path = 'green.jpg'
# red_image_path = 'red.jpg'

# red_image = cv2.imread(green_image_path)
# green_image_path = cv2.imread(green_image_path)

color = {'red': (0, 0, 255), 'green': (0, 255, 0), 'yellow': (0, 255, 255)}

current_car_light = color['red']

import serial

arduino = serial.Serial(port='/dev/cu.usbmodem212201', baudrate=115200)


green_and_red_time = 3 #绿灯/红灯维持时间
yellow_time = 1



target_car_light = color['green']

# color = {'red': (255, 0, 0), 'green': {0, 255, 0}, 'yellow':}

def change_to_green():
    arduino.write(bytes('G', 'utf-8'))
    global current_car_light
    current_car_light = color['green']

def change_to_red():
    arduino.write(bytes('R', 'utf-8'))
    global current_car_light
    current_car_light = color['red']
    
def change_to_yellow():
    arduino.write(bytes('Y', 'utf-8'))
    global current_car_light
    current_car_light = color['yellow']



time.sleep(3)

while True:
    if not time_interv_started:
        time_interv_start_time = time.time()
        time_interv_started = True
        

    ret, frame = capture.read()
    if not ret:
        break


    results = model.track(frame, persist=True, tracker='bytetrack.yaml', conf=0.1, verbose=False)
    
    car_results = car_model.track(frame, persist=True, tracker='bytetrack.yaml', conf=0.1, verbose=False)
    #YOLO检测
    for result in results:
        boxes = result.boxes #检测到物件

        for box in boxes:
            cls = int(box.cls[0].item())
            conf = round(float(box.conf[0].item()), 2)

            if cls in car_classes or cls == people_class: #当物件是人或一种交通工具
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = x2-x1, y2-y1
                bbox = (x1, y1, w, h)
                cvzone.cornerRect(frame, bbox) #标记交通工具或人

                id = box.id

                if id is not None:
                    id = int(id.item()) #用来分辨每一个人与车

                            
                    if cls == people_class:
                        detected_people_id.add(id)
                        if id not in global_people_start_time:
                            global_people_start_time[id] = time.time()
                            
                        else:
                            wait_time = time.time()-global_people_start_time[id]
                            cvzone.putTextRect(frame, str(round(wait_time, 1)), (x1, y1))
                            if wait_time >= wait_time_bound:
                                waiting_people_id.add(id)
                                
    for car_result in car_results:
        boxes = car_result.boxes #检测到物件

        for box in boxes:
            cls = int(box.cls[0].item())
            conf = round(float(box.conf[0].item()), 2)

            if cls in car_classes: #当物件是人或一种交通工具
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = x2-x1, y2-y1
                bbox = (x1, y1, w, h)
                cvzone.cornerRect(frame, bbox) #标记交通工具或人

                id = box.id

                if id is not None:
                    id = int(id.item()) #用来分辨每一个人与车
                
                    if cls in car_classes: #如果是交通工具
                        detected_car_id.add(id)
                        if id not in global_car_start_time: #若还没有开始计算等待时间，就记录车的开始等待时间
                            global_car_start_time[id] = time.time() 
                    
                        else: #否则：计算等待了多久
                            wait_time = time.time()-global_car_start_time[id]
                            cvzone.putTextRect(frame, str(round(wait_time, 1)), (x1, y1))
                            if wait_time >= wait_time_bound and id not in waiting_car_id: #若等待了一会儿，就把该车辆id分类为正在等待车辆
                                waiting_car_id.add(id)
                        
    #写下等了多久人与车
    cvzone.putTextRect(frame, f'Waiting people: {len(waiting_people_id)}', (0, 300))
    cvzone.putTextRect(frame, f'Waiting car: {len(waiting_car_id)}', (0, 100))
    
    if current_car_light == color['yellow']:
        if time.time()-changed_light_time >= 2: #Over time

            if target_car_light == color['red']:
                change_to_red()
                changed_light_time = time.time()

                
            else: #Target light is green
                change_to_green()
                changed_light_time = time.time()

    
    #计算比例
    if time.time()-time_interv_start_time >= recalc_time_interv:

        #避免比例是0或者无法计算（ZeroDivisionError)
        waiting_people = len(waiting_people_id)
        waiting_car = len(waiting_car_id)
        
        try:
            car_people_ratio = waiting_car/waiting_people
        except:
            car_people_ratio = math.inf

        #若比例有到一定数值，and haven't changed light for a while, then change
        if car_people_ratio >= car_green_light_bound and time.time()-changed_light_time >= shortest_dur:
            if current_car_light == color['red']: #Only define as light changed if it is previously red and I change it to green
                changed_light_time = time.time()
                current_car_light = color['yellow']
                target_car_light = color['green']
                change_to_yellow()            
            
        #Or else keep green
        elif car_people_ratio < car_red_light_bound and  time.time() - changed_light_time >= shortest_dur:
            if current_car_light == color['green']:
                changed_light_time = time.time()
                change_to_yellow()
                target_car_light = color['red']
            

        global_car_start_time = {key: value for key, value in global_car_start_time.items() if key in detected_car_id}
        global_people_start_time = {key: value for key, value in global_people_start_time.items() if key in detected_people_id}
            
        time_interv_started = False
        waiting_people = 0
        waiting_car = 0
        waiting_people_id = set()
        waiting_car_id = set()
        detected_people_id = set()
        detected_car_id = set()
        
    #如果绿灯时间过长，换成红灯
    if time.time()-changed_light_time >= longest_dur:
        current_car_light = color['red'] if current_car_light == color['green'] else 'green'
        changed_light_time = time.time()

    cvzone.putTextRect(frame, f'Ratio: {car_people_ratio}', (0, 500))
    
    
    cv2.rectangle(frame, (0, 700), (380, 1080),current_car_light, -1)  
    
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) == ord('d'):
        break


capture.release()
cv2.destroyAllWindows()