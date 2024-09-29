import serial
import time

port = None #请输入arduino的port
arduino = serial.Serial(port, 115200)

current_light = 'G'


green_and_red_time = 3 #绿灯/红灯维持时间
yellow_time = 1

green_start_time = None
red_start_time = None
yellow_start_time = None

def change_to_green():
    arduino.write(bytes('G', 'utf-8'))
    global current_light
    current_light = 'G'

def change_to_red():
    arduino.write(bytes('R', 'utf-8'))
    global current_light
    current_light = 'R'
    
def change_to_yellow():
    arduino.write(bytes('Y', 'utf-8'))
    global current_light
    current_light = 'Y'


change_to_green()
green_start_time = time.time()
time.sleep(3)
target_light = 'R'

while True:
    if (current_light == 'G'):
        if time.time()-green_start_time >= green_and_red_time:
            change_to_yellow()
            yellow_start_time = time.time()
            
        #Or else keep green
        
        
    if current_light == 'Y':
        if time.time()-yellow_start_time >= yellow_time: #Over time

            if target_light == 'R':
                change_to_red()
                red_start_time = time.time()
                target_light = 'G'
                
            else: #Target light is green
                change_to_green()
                target_light = 'R'
                green_start_time = time.time()
                
    if (current_light == 'R'):
        if time.time()-red_start_time >= green_and_red_time:
            change_to_yellow()
            yellow_start_time = time.time()
            
        #Or else keep green
                
    
                    
    
    
    
    

    
    