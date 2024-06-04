from utils.brick import Motor,  wait_ready_sensors, reset_brick, BP, EV3UltrasonicSensor, EV3ColorSensor, TouchSensor
import time
from time import sleep
import math
from utils import brick
from cmath import isclose
import threading

reset_brick()
OPEN_TUNNEL = 1
INSIDE_TUNNEL = False
RED_DETECTED = False
LEFT_MOTOR = Motor("D")
RIGHT_MOTOR = Motor("A")
LOADER = Motor("B")
FLY_WHEELS = Motor("C")
TOUCH_SENSOR = TouchSensor(3)
RIGHT_US = EV3UltrasonicSensor(4) # S1
LEFT_US = EV3UltrasonicSensor(1) # S1
CS = EV3ColorSensor(2) # S2
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.0925 #0.835 #0.57 #0.148
SPEED_LIMIT = 500
ORIENT_TO_DEG = AXLE_LENGTH / WHEEL_RADIUS
DIST_TO_DEG = 180 / (math.pi*WHEEL_RADIUS)

#WALL FOLLOWER
SAMPLING_INTERVAL = 0.3 # Sampling Interval 200ms or 5Hz
DEADBAND = 0.003 # Deadband is 2cm
US_OUTLIER = 200 # Ignore ultrasonic readings > 200
POWER_LIMIT = 100 # Motor Power limit = 80%
SPEED_LIMIT = 720 # Motor Speed limit = 720dps
    
STOP_BOOL = False
TUNNEL = False

def move(distance, speed):
    try:
        distance = distance*0.30
        LEFT_MOTOR.set_dps(speed)
        RIGHT_MOTOR.set_dps(1.15*speed)
        LEFT_MOTOR.set_limits(SPEED_LIMIT, speed)
        RIGHT_MOTOR.set_limits(SPEED_LIMIT, speed)
        LEFT_MOTOR.set_position_relative(int(distance * DIST_TO_DEG))
        RIGHT_MOTOR.set_position_relative(int(distance * DIST_TO_DEG))
        wait_for_motor(RIGHT_MOTOR)
    except Exception as e:
        print("ERROR: ", e)
def wait_for_motor(motor: Motor):
    try:
        while math.isclose(motor.get_speed(), 0):
            time.sleep(0.02)
        while not math.isclose(motor.get_speed(), 0):
            time.sleep(0.02)
    except Exception as e:
        print("ERROR: ", e)
    
def rotate(angle, speed):
    try:
        LEFT_MOTOR.reset_encoder()
        RIGHT_MOTOR.reset_encoder()
        LEFT_MOTOR.set_dps(speed)
        RIGHT_MOTOR.set_dps(speed)
        LEFT_MOTOR.set_limits(SPEED_LIMIT, speed)
        RIGHT_MOTOR.set_limits(SPEED_LIMIT, speed)
        LEFT_MOTOR.set_position_relative(int(angle * ORIENT_TO_DEG))
        RIGHT_MOTOR.set_position_relative(-int(angle * ORIENT_TO_DEG))
        wait_for_motor(RIGHT_MOTOR)
    except Exception as e:
        print("ERROR: ", e)
        
def stop():
    LEFT_MOTOR.set_power(0)
    RIGHT_MOTOR.set_power(0)

def detect_tunnel():
    global STOP_BOOL
    global OPEN_TUNNEL
    try:
        while not STOP_BOOL:
            if (TOUCH_SENSOR.is_pressed()):
                STOP_BOOL = True
                OPEN_TUNNEL = 2
                return
        sleep(0.1)
    except Exception as e:
        print("ERROR: ", e)

    
def identify_launch_zone():
    global STOP_BOOL
    while True:
        colour = CS.get_rgb()
        #print(colour)
        if (colour[0] is not None and colour[1] is not None and colour[2] is not None and colour[1] < 40 and colour[2] < 30 and colour[0] > 0 and colour[1] > 0 and colour[2] > 0):
            STOP_BOOL = True
            print("red detected")
            return
        time.sleep(0.05)
        
def check_if_inside_tunnel():
    print("i am check if inside tunnel")
    global STOP_BOOL
    global OPEN_TUNNEL
    global TUNNEL
    try:
        while True:
            if STOP_BOOL == True:
                print("leaving inside tunnel loop")
                return
            colour = CS.get_rgb()

            if (colour[0] is not None and colour[1] is not None and colour[2] is not None and colour[0] > 90 and colour[0] < 120 and colour[1] > 80 and colour[1] < 105 and colour[2] < 70 and colour[2] > 50 and colour[0] > 0 and colour[1] > 0 and colour[2] > 0):
                STOP_BOOL = True
                print('Inside tunnel')
                TUNNEL = True
                return
            if OPEN_TUNNEL == 1:
                if (TOUCH_SENSOR.is_pressed()):
                    print("TS PRESSED")
                    STOP_BOOL = True
                    OPEN_TUNNEL = 2
                    return
            time.sleep(0.01)
    except Exception as e:
        print("ERROR: ", e)
        
def check_if_outside_tunnel():
    global STOP_BOOL
    print("i am check if outside tunnel")
    try:
        while True:
            colour = CS.get_rgb()

            if (colour[0] is not None and colour[1] is not None and colour[2] is not None and colour[0] > 110 and colour[0] < 140 and colour[1] > 150 and colour[1] < 175 and colour[2] < 170 and colour[2] > 130 and colour[0] > 0 and colour[1] > 0 and colour[2] > 0):
                STOP_BOOL = True
                TUNNEL = False
                print('Outside tunnel')
                return
            time.sleep(0.25)
        
    except Exception as e:
        print("ERROR: ", e)
        
def check_if_at_loading_zone():
    global STOP_BOOL
    try:
        while True:
            if (TOUCH_SENSOR.is_pressed()):
                STOP_BOOL = True
                print('REACHED LOADING ZONE')
                return
            time.sleep(0.01)
    except Exception as e:
        print("ERROR: ", e)
        
def move_forever():
    global RED_DETECTED
    while True:
        LEFT_MOTOR.set_power(-60)
        RIGHT_MOTOR.set_power(-60)
        time.sleep(0.1)
        if RED_DETECTED:
            print("stop")
            stop()
            return

def follow_wall(wall_dist, fwd_speed, kp, ki, kd, deadband, us_sensor):
    global STOP_BOOL
    try:
        wait_ready_sensors() # Wait for sensor initialization
        # Set motor power and speed limits
        LEFT_MOTOR.set_limits(POWER_LIMIT, SPEED_LIMIT)
        RIGHT_MOTOR.set_limits(POWER_LIMIT, SPEED_LIMIT)
        # Reset motor encoders to 0 value
        LEFT_MOTOR.reset_encoder()
        RIGHT_MOTOR.reset_encoder()
        
        integral = 0
        last_error = 0
        
        LEFT_MOTOR.set_dps(fwd_speed) # Set the motor speeds to start them
        RIGHT_MOTOR.set_dps(fwd_speed)
        
        while True:
            if STOP_BOOL:
                return
                
            print("wall following")
            dist = us_sensor.get_cm() # Get distance reading from wall
            print("DISTANCE ", dist)
            if dist >= US_OUTLIER: # If error or too far, no error correction
                dist = wall_dist
                
            dist = dist / 100.0 # Convert to meters
            error = wall_dist - dist # Get error difference
            print('dist: {:0.3f}'.format(dist))
            print('error: {:0.3f}'.format(error))
            
            integral += error * SAMPLING_INTERVAL
            derivative = (error - last_error) / SAMPLING_INTERVAL
            
            output = kp * error + ki * integral + kd * derivative
            last_error = error
            
            # Error within Deadband: wheels rotate same speed, go forward
            if abs(error) <= deadband:
                if TUNNEL:
                    LEFT_MOTOR.set_dps(fwd_speed)
                    RIGHT_MOTOR.set_dps(fwd_speed)
                else:
                    LEFT_MOTOR.set_dps(fwd_speed*1.5)
                    RIGHT_MOTOR.set_dps(fwd_speed*1.5)
                print('reaction: no correction')
            else:
                if us_sensor == LEFT_US:
                    left_speed = fwd_speed
                    right_speed = fwd_speed + output
                else:
                    left_speed = fwd_speed + output
                    right_speed = fwd_speed
                    
                LEFT_MOTOR.set_dps(left_speed)
                RIGHT_MOTOR.set_dps(right_speed)
                print('reaction: correcting, left_speed: {}, right_speed: {}'.format(left_speed, right_speed))
                
            time.sleep(SAMPLING_INTERVAL) # Sleep for sampling interval
            
    except Exception as e:
        print("ERROR: ", e)
        
def indicate_readiness():
    for power in range(0, 40, 1): # go max speed to make sure wheels start spinning properly
        FLY_WHEELS.set_power(power)
        time.sleep(0.05)
    time.sleep(0.5)
    for power in range(40, 0, -1): # go max speed to make sure wheels star spinning properly
        FLY_WHEELS.set_power(power)
        time.sleep(0.05)
    time.sleep(0.5)
    FLY_WHEELS.set_power(0)
        
def launch():
    # Launching values: 55, 65, 81
    # 45 for dispenser testing
    FS1 = 55 #speed for target 1 
    FS2 = 69
    FS3 = 81
    #FS4 = 77
    LOADUP = 10 #speed for loading, change time if not work
    LOADDOWN = -20
    counter = 0

    for power in range(0, 100, 1): # go max speed to make sure wheels start spinning properly
        FLY_WHEELS.set_power(power)
        time.sleep(0.05)
    time.sleep(0.5)
    for power in range(99, FS1, -1): # Go to first basket speed
        FLY_WHEELS.set_power(power)
        time.sleep(0.05)
    time.sleep(2)

    for _ in range(2):
        LOADER.set_power(LOADUP) 
        time.sleep(1.05) 
        LOADER.set_power(LOADDOWN)
        time.sleep(1)
        counter+=1
        if counter%2==0:
            wiggle()
        else:
            time.sleep(0.5)
        
    time.sleep(1)

    for power in range(FS1, FS2, 1):  # Accelerate to second basket speed
        FLY_WHEELS.set_power(power)  
        time.sleep(0.05)
    time.sleep(2)
        
    for _ in range(3):
        LOADER.set_power(LOADUP) 
        time.sleep(1.05) 
        LOADER.set_power(LOADDOWN)
        time.sleep(1)
        counter+=1
        if counter%2==0:
            wiggle()
        else:
            time.sleep(0.5)
        
    time.sleep(0.5)
        
    for power in range(FS2, FS3, 1):  # Accelerate to third basket sped first try
        FLY_WHEELS.set_power(power)  
        time.sleep(0.05)
    time.sleep(2)
    
    for _ in range(12):
        LOADER.set_power(LOADUP) 
        time.sleep(1.05) 
        LOADER.set_power(LOADDOWN)
        time.sleep(1)
        counter+=1
        if counter%2==0:
            wiggle()
        else:
            time.sleep(0.5)
    time.sleep(1)

    for power in range(FS3, 0, -1):  # Decelerate
        FLY_WHEELS.set_power(power) 
        time.sleep(0.05)
        
    FLY_WHEELS.set_power(0)
    LOADER.set_power(0)
    
def wiggle():

    print("wiggle")
    move(-0.15, 800)
    time.sleep(0.2)
    move(0.11, 800)
    rotate(1 ,800)


if __name__ == "__main__":
    wait_ready_sensors()
    LEFT_MOTOR.reset_encoder()
    RIGHT_MOTOR.reset_encoder()
    
    # on our way to the loading zone 
    detect_if_enter_tunnel_thread1 = threading.Thread(target=check_if_inside_tunnel)
    detect_tunnel_thread1 = threading.Thread(target=detect_tunnel)
    # wall_dist, fwd_speed, kp, ki, kd, deadband, us_sensor
    wall_thread1 = threading.Thread(target=follow_wall, args=(0.357, ), kwargs={'fwd_speed': 300, 'kp': 2000, 'ki': 10, 'kd': 1500,'deadband' : 0.003, 'us_sensor' : RIGHT_US})
    
    
    detect_if_enter_tunnel_thread1_1 = threading.Thread(target=check_if_inside_tunnel)
    detect_tunnel_thread1_1 = threading.Thread(target=detect_tunnel)    
    wall_thread1_1 = threading.Thread(target=follow_wall, args=(0.04, ), kwargs={'fwd_speed': 300, 'kp': 2500, 'ki': 10, 'kd': 1500,'deadband' : 0.003, 'us_sensor' : RIGHT_US})
  
    detect_if_exit_tunnel_thread = threading.Thread(target=check_if_outside_tunnel)
    wall_thread2 = threading.Thread(target=follow_wall, args=(0.046, ), kwargs={'fwd_speed': 150, 'kp': 2000, 'ki': 10, 'kd': 1500,'deadband' : 0.003, 'us_sensor' : RIGHT_US})
    
    wall_thread3 = threading.Thread(target=follow_wall, args=(0.213, ), kwargs={'fwd_speed': 800, 'kp': 2000, 'ki': 10, 'kd': 1500,'deadband' : 0.01, 'us_sensor' : RIGHT_US})
    detect_loading_zone_thread = threading.Thread(target=check_if_at_loading_zone)
    
    # on our way to the launching zone
    wall_thread4 = threading.Thread(target=follow_wall, args=(0.2, ), kwargs={'fwd_speed': 700, 'kp': 2000, 'ki': 10, 'kd': 1500,'deadband' : 0.01, 'us_sensor' : LEFT_US})
    detect_loading_zone_thread2 = threading.Thread(target=check_if_at_loading_zone)
    
    # tunnel 1
    wall_thread5_1 = threading.Thread(target=follow_wall, args=(0.347, ), kwargs={'fwd_speed': 300, 'kp': 2000, 'ki': 10, 'kd': 1500,'deadband' : 0.01, 'us_sensor' : LEFT_US})
    # tunnel 2
    wall_thread5_2 = threading.Thread(target=follow_wall, args=(0.04, ), kwargs={'fwd_speed': 300, 'kp': 2000, 'ki': 10, 'kd': 1500, 'deadband' : 0.003, 'us_sensor' : LEFT_US})
    detect_if_enter_tunnel_thread2 = threading.Thread(target=check_if_inside_tunnel)
    #inside tunnel
    wall_thread6 = threading.Thread(target=follow_wall, args=(0.046, ), kwargs={'fwd_speed': 150, 'kp': 2000, 'ki': 10, 'kd': 1500, 'deadband' : 0.003, 'us_sensor' : LEFT_US})
    detect_if_exit_tunnel_thread2 = threading.Thread(target=check_if_outside_tunnel)
    
    #going to launching zone
    wall_thread7 = threading.Thread(target=follow_wall, args=(0.213, ), kwargs={'fwd_speed': 400, 'kp': 2000, 'ki': 10, 'kd': 1500, 'deadband' : 0.01, 'us_sensor' : LEFT_US})
    wall_thread7_1 = threading.Thread(target=follow_wall, args=(0.357, ), kwargs={'fwd_speed': 400, 'kp': 2000, 'ki': 10, 'kd': 1500, 'deadband' : 0.01, 'us_sensor' : LEFT_US})

    colour_sensor = threading.Thread(target=identify_launch_zone)
    
    FLY_WHEELS.float_motor()
    LOADER.float_motor()
    
    #START
    time.sleep(0.2)
    rotate(28, 700)
    time.sleep(0.2)
    move(1.1, 700)
    time.sleep(0.2)
    rotate(-28, 700)
    time.sleep(0.2)
    
    # detecting which tunnel is open
    wall_thread1.start()
    detect_if_enter_tunnel_thread1.start()
    wall_thread1.join()
    detect_if_enter_tunnel_thread1.join()
    
    STOP_BOOL = False

    if OPEN_TUNNEL == 2:
        time.sleep(1)
        print("move")
        move(-4, 600)
        time.sleep(0.7)
        print("rotate")
        rotate(-40, 700)
        time.sleep(0.7)
        print("move")
        move(1.6, 700)
        print("rotate")
        time.sleep(0.7)
        rotate(35, 700)
        time.sleep(0.7)
        wall_thread1_1.start()
        detect_if_enter_tunnel_thread1_1.start()
       
        wall_thread1_1.join()
        detect_if_enter_tunnel_thread1_1.join()
         
        STOP_BOOL = False
    
    move(0.5, 300)
    wall_thread2.start()
    detect_if_exit_tunnel_thread.start()
    wall_thread2.join()
    detect_if_exit_tunnel_thread.join()
    
    # exited the tunnel
    
    STOP_BOOL = False
    time.sleep(0.2)
    move(1.5, 300)
    time.sleep(0.2)
    rotate(90, 300)
    time.sleep(0.2)
    
    wall_thread3.start()
    detect_loading_zone_thread.start()
    wall_thread3.join()
    detect_loading_zone_thread.join()
    STOP_BOOL = False
    time.sleep(0.2)
    stop()
    time.sleep(0.2)
    STOP_BOOL = False
    move(-0.2,300)
    rotate(170, 600)
    move(-0.25,300)
    
    
    indicate_readiness()
    
    #reached loading zone
    
    while True:
        value = RIGHT_US.get_cm()
        if (value < 5 and value > 0):
            time.sleep(0.1)    
            break   
        time.sleep(0.1)
 
    wall_thread4.start()
    detect_loading_zone_thread2.start()
    wall_thread4.join()
    detect_loading_zone_thread2.join()
    
    
    
    STOP_BOOL = False
    
    if OPEN_TUNNEL == 2:
        move(-0.35, 300)
        time.sleep(0.5)
        rotate(-100, 300)
        time.sleep(0.5)
        move(-0.7, 300)
        time.sleep(0.5)
        wall_thread5_2.start()
        detect_if_enter_tunnel_thread2.start()
        wall_thread5_2.join()
        detect_if_enter_tunnel_thread2.join()
        
    else:
        move(-0.95, 300)
        time.sleep(0.5)
        rotate(-82, 300)
        time.sleep(0.5)
        move(-0.60, 300)
        time.sleep(0.5)
        wall_thread5_1.start()
        detect_if_enter_tunnel_thread2.start()
        wall_thread5_1.join()
        detect_if_enter_tunnel_thread2.join()
    
    # entered tunnel

    STOP_BOOL = False
    move(0.5, 300)
    wall_thread6.start()
    detect_if_exit_tunnel_thread2.start()
    wall_thread6.join()
    detect_if_exit_tunnel_thread2.join()

    # exited tunnel
    
    STOP_BOOL = False
    colour_sensor.start()
    if OPEN_TUNNEL == 1:
        move(1.5, 400)
        rotate(35, 300)
        move(1, 400)
        rotate(-35, 300)
    else:
        move(1.5, 400)
        rotate(-35, 300)
        move(1, 400)
        rotate(35, 300)
    
    # reached launch zone
    wall_thread7.start()
    colour_sensor.join()
    wall_thread7.join()
    
    STOP_BOOL = False
    
    move(0.5, 400)
    rotate(-80, 400)

    launch()
    stop()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    


