####### Imports #######
import network  
import machine  
import time     
import socket
import uasyncio

####### Global Variables #######
SSID = "mu2"        # WiFi SSID   
PASS = "05690569"   # WiFi Password


####### GPIOs #######
but_override = machine.Pin(14, machine.Pin.IN)       # BUTTON - Push to Override
door_lock = machine.Pin(13, machine.Pin.IN)          # BUTTON - Reed switch: Door Locked - LOW

led_lock = machine.Pin(5, machine.Pin.OUT)           # LED - ON: When unlocked
led_override = machine.Pin(16, machine.Pin.OUT)      # LED - ON: When Override Requested
led_server = machine.PWM(machine.Pin(0), 5000)       # LED - ON: when connected to server
led_wifi = machine.PWM(machine.Pin(12), 5000)        # LED - ON: When connected to WiFi
led_notify = machine.Pin(15, machine.Pin.OUT)        # LED - ON: On notification 

servo = machine.PWM(machine.Pin(4, machine.Pin.OUT)) # Servo Object: Unlock Door

####### Global Variables #######
lock_state = True        # State of the Lock
pos_lock = 60            # Angle of servo to Lock
pos_unlock = 160         # Angle of servo to unlock
prev_unlock_time = 0     # Previous Lock time

connect  = ""            # Empty variable to store WiFi connect object
server = ""              # Empty variable to store the connection to Server
is_connected = False

# Implementing a map function (0, 180) degrees --> (20, 120) duty values
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def pwr_restart():
    global pos_unlock, pos_lock, door_lock
    circle_servo(pos_unlock)
    led_lock.value(1)
    time.sleep(1)
    led_lock.value(0)
    time.sleep(1)
    led_lock.value(1)
    time.sleep(1)
    led_lock.value(0)
    time.sleep(1)
    led_lock.value(1)
    time.sleep(1)
    led_lock.value(0)
    time.sleep(1)
    is_locked = True
    while is_locked:
        led_lock.value(1)
        time.sleep(1)
        led_lock.value(0)
        time.sleep(1)
        is_locked = door_lock.value()
    led_override.value(1)
    led_lock.value(1)
    circle_servo(pos_lock)
    machine.reset()

def connect2WiFi():
    """Connect to WiFi using global SSID and Password"""
    global connect, led_wifi 
    connect = network.WLAN(network.STA_IF) # Setting System to Station Mode
    print("Connecting to WiFi")
    connect.active(True)
    connect.connect(SSID, PASS) # Connecting to the network
    while not connect.isconnected():
        fade(led_wifi) # Fade the led, when not connected
        print(".", end="")
    print("Connected to WiFi")
    print(connect.ifconfig()) # Print the IP address of the ESP8266
    led_wifi.duty(1024) # Turn on the WIFILED

def fade(led, d=0.005):
    for v in range(0, 1024):
        led.duty(v)
        time.sleep(d)
    for v in range(0, 1024):
        led.duty(1024 - v)
        time.sleep(d)

def connect2Server():
    global server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server.
    is_connected = False
    while not is_connected:
        try:
            fade(led_server, 0.003)
            print(".", end="")
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(server.connect(("192.168.1.100", 8080)))
            led_server.duty(1024)
            is_connected = True
        except Exception as e:
            print("73:", e)
            fade(led_server, 0.003)
    try:
        server.send("TEST".encode())
    except Exception as e:
        connect2Server()
        print ("From Line 79:", e)
    
def check_message():
    global server
    server.settimeout(5.0)
    msg = ""
    try:
        msg = server.recv(1024)
        msg = msg.decode()
    except Exception as e:
        print("89:", e)
    if(len(msg)) > 1:
        print("Recieved message:", msg)
        if msg == "OPEN":
            server.send("OK".encode())
            unlock()
        elif msg == "TEST":
            server.send("OK".encode())
        elif msg == "NOTIFY":
            led_notify.value(1)
        elif msg == "OVERRIDE":
            override(1)
        elif msg == "SHUTDOWN":
            pwr_restart()
        else:
            print("Unknown message")
            server.send("MSG_UNKOWN".encode())

def circle_servo(angle):
    """
    Control Servo Rotation

    Parameters:
    angle (int): An angle from 0 to 180 degree

    Return: NULL
    """
    global servo # Using global servo object
    print("Rotating Servo")
    servo.duty(map(angle, 0, 180, 20, 120)) # Using the mapped value to change the servo angle
    time.sleep(2) # Sleep for 2 seconds 
    servo.duty(0)

def setup():
    """Setup the system and set the defaults"""
    global connect, led_wifi, servo, led_lock, led_override, led_server, led_wifi
    led_wifi.duty(0)
    led_server.duty(0)
    led_lock.value(0)
    led_override.value(0)
    time.sleep(1)
    led_wifi.duty(1024)
    time.sleep(1)
    led_server.duty(1024)
    time.sleep(1)
    led_override.value(1)
    time.sleep(1)
    led_lock.value(1)
    time.sleep(1)
    led_notify.value(1)
    time.sleep(1)
    led_wifi.duty(0)
    led_server.duty(0)
    led_lock.value(0)
    led_override.value(0)
    led_notify.value(0)
    # Handling servo PWM configuration
    servo.freq(50)
    servo.duty(0)
    if door_lock.value():
        circle_servo(pos_unlock)
    else:
        circle_servo(pos_lock)
    # Configuring interrupt to handle override
    but_override.irq(trigger=machine.Pin.IRQ_RISING, handler=override)
    connect2WiFi()      # Connect to WiFi
    connect2Server()    # Conenct to Server
    print("End of Setup")

def check_wiFi():
    # TODO: Function testing to be done
    """Check if ESP8266 is connected to WiFi"""
    global connect, led_server
    if connect.isconnected():
        print(connect.ifconfig())
        led_server.duty(0)
        connect2WiFi()

def check_server():
    """Check if the ESP8266 is connected to the Server"""
    global server
    try:      
        server.send("TEST".encode())
    except Exception as e:
        print("Checking Server, unable to comunicate")
        check_wiFi()
        # server.connect(("192.168.1.100", 8080))
        connect2Server()

def loop():
    """Keeps Running as long as the system is powered"""
    check_server()
    check_message()


def override(p):
    """Override System defaults, and open the latch"""
    print("Overriding")
    global lock_state, prev_unlock_time, pos_unlock, pos_lock, led_override
    print(prev_unlock_time - time.time())
    if  time.time() - prev_unlock_time > 5:
        prev_unlock_time = time.time()
        print("Time matching")
        if lock_state == False:
            lock_state = True
            circle_servo(pos_lock)
            print("Locking")
            led_override.value(0) # Notifying that System is overriden
        else:
            lock_state = False
            led_notify.value(0)
            circle_servo(pos_unlock)
            print("Unlocking")
            led_override.value(1) # Notifying that System is out of overriden
   
# Defining a fuction to lock/unlock + LED
# @author: Rishil
# @date: 28-12-2021
def unlock():
    """Unlatch the door"""
    # global led_lock, servo, pos_unlock, door_lock
    circle_servo(pos_unlock) # Unlocking the door
    led_notify.value(0)
    led_lock.value(1)
    time.sleep(1)            # Keep the lock unlocked for a few seconds
    is_locked = True
    while is_locked:
        time.sleep(1)
        is_locked = door_lock.value()
    circle_servo(pos_lock)   # Lock back the door
    led_lock.value(0)

setup()     # Run the setup function once to setup the environment and the variables
while True:
    loop()  # Keep running the loop
