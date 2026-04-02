'''
******READ BEFORE RUNNING*******
Summary of Code:
This code runs the IMU and GPS code simultaneously and displays their corresponding data.

If Running Code on Raspberry Pi Pico For First Time:
Only one Pi has all of the necessary installed and therefore is fully functional.
Working on the others will not work unless all of the libraries are there. See the 
transition document for more information on this.
There is also a specific wiring needed for everything,
this would be in the transition document.

Thing To Know When Modifying Code:
This code is running on MicroPython (not CircuitPython - they're slightly different),
therefore any modifications must be done in MicroPython. It's not much different than
regular Python, it's just the implementation of modules that's wonky. Since it is running
on MicroPython, only one file can be run at a time, so everything must be located here.
You can't reference multiple files, it will not work.

Why did we choose MicroPython instead of CircuitPython?:
We needed access to double precision values for the GPS coordiantes contained in this document. 
MicroPython and CircuitPython both use floating point values as a default, which causes not enough precision
when using coordinates. This can cause graphs to show a "grid" pattern. To use double precision instead of floating point,
you can modify MicroPython's source files before flashing them to a microcontroller. See Drew Fowler or Aadhavan Srinivasan's
notebooks from Spring 2024 for instructions and more information.

Current Bugs:
Sometimes there are "zero" values read by the GPS sensor (24/10,405 points in most recent test).
Number of IMU update points and IMU refresh rate need to be optimized.
Could also fix FCD screen messages, although these messages are primarily for testing purposes - final deliverable will not have an LCD screen.

Contact Info:
Drew Fowler: fowler52@purdue.edu / anfowler2001@gmail.com
Aadhavan Srinivasan: srini193@purdue.edu
'''
'''
import time
import busio
import board
import adafruit_bno055
from machine import Pin, UART, I2C
# import supervisor

import math

import machine
import select
import sys

'''
import time
import math
from machine import Pin, UART, I2C
import bno055  # Standard MicroPython BNO055 driver (needs to be added to Pico)
import select
import sys






INT_MAX = 10000

# Creates coordinate point struct
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def get_latitude(value, dir):
    latDeg = float(value[0: 2])
    latMin = float(value[2: 10]) / 60
    latitude = (float(latDeg) + float(latMin))
    if dir == "S":
        latitude = -latitude
    return latitude


# Gets Longitude
def get_longitude(value, dir):
    longDeg = float(value[0: 3])
    longMin = float(value[3: 11]) / 60
    longitude = (float(longDeg) + float(longMin))
    if dir == "W":
        longitude = -longitude
    return longitude

# To determine if the coordinate/function q lies on the segment pr
def onSegment(p:tuple, q:tuple, r:tuple) -> bool:
    
    if((q[0] <= max(p[0], r[0])) &
       (q[0] >= min(p[0], r[0])) &
       (q[1] <= max(p[1], r[1])) &
       (q[1] >= min(p[1], r[1]))):
        return True
    return False

#Finding orientation
def orientation(p:tuple, q:tuple, r:tuple) -> int:
    
    val = (((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))) #calculating slope
    
    if (val > 0):
        return 1 #positive slope is clockwise orientation
    elif (val < 0):
        return 2 #negative slope is counterclockwise orientation
    else:
        return 0 #collinear orientation
    

#Determine if line segment p1q1 and p2q2 intersects
def doIntersect(p1, q1, p2, q2):
    
    #looking for orientation
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    
    #General Case: If the orientations are different, they intersect
    if (o1 != o2) and (o3 != o4):
        return True
    
    #Special Case (collinear): If x and y projection intersects, they intersect
    if (o1 == 0) and (onSegment(p1, p2, q1)): #p1, q1 and p2 are collinear and p2 lies on segment p1q1
        return True
    
    if (o2 == 0) and (onSegment(p1, q2, q1)): #p1, q1 and q2 are collinear and q2 lies on segment p1q1
        return True
    
    if (o3 == 0) and (onSegment(p2, p1, q2)): #p2, q2 and p1 are collinear and p1 lies on segment p2q2
        return True
    
    if (o4 == 0) and (onSegment(p2, q1, q2)): #p2, q2 and q1 are collinear and q1 lies on segment p2q2
        return True
    
    return False


#Determine if the point p lies within the polygon
def is_within_polygon(points:list, p:list) -> bool:
    
    n = len(points)
    if n < 3: #there must be at least 3 points/vertices in a polygon
        return False
    
    extreme = (INT_MAX, p[1]) #Create a point for line segment from p to infinite
    
    decrease = 0 #To calculate number of points where y-coordinate of the polygon is equal to y-coordinate of the point
    count = i = 0
    
    while True:
        next = (i + 1) % n
        
        if(points[i][1] == p[1]):
            decrease += 1
            
        if (doIntersect(points[i], points[next], p, extreme)):
            if orientation(points[i], p, points[next]) == 0:
                return onSegment(points[i], p, points[next])
                           
            count += 1
                           
        i = next
        
        if (i == 0):
            break
        
        count -= decrease
        
    return (count % 2 == 1)

# Turns on the LCD
def initialize_lcd(backlight_red, backlight_green, backlight_blue):
    # MicroPython uses UART and Pin from the machine module
    lcd_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
    lcd_uart.write(b'|')  
    lcd_uart.write(b'\x18')  
    lcd_uart.write(b'\x08')  
    lcd_uart.write(b'|')  
    lcd_uart.write(b'\x2B')  
    lcd_uart.write(backlight_red.to_bytes(1, 'big'))  
    lcd_uart.write(backlight_green.to_bytes(1, 'big'))  
    lcd_uart.write(backlight_blue.to_bytes(1, 'big')) 
    lcd_uart.write(b'|')  
    lcd_uart.write(b'-')  
    return lcd_uart

# Initializes GPS
def initialize_gps():
    # rxbuf is the MicroPython equivalent of receiver_buffer_size
    return UART(0, baudrate=38400, tx=Pin(12), rx=Pin(13), bits=8, stop=1, timeout=10, rxbuf=512)

            
# Gets Current Location
def get_current_location(gps_uart):
    
    # Loop continuously until we get a valid satellite fix
    while True:
        line = gps_uart.readline()
        
        if line:
            # Decode the line, ignoring garbage bytes from electrical noise
            line = line.decode('ascii', 'ignore').strip()
            
            # Look specifically for the $GNGGA sentence
            if line.startswith('$GNGGA'):
                parts = line.split(',')
                
                # Safety check: Ensure the sentence isn't cut off AND has a valid lock (parts[6] != '0')
                if len(parts) > 6 and parts[6] != '0':  
                    
                    lat = parts[2]
                    lat_dir = parts[3]
                    lon = parts[4]
                    lon_dir = parts[5]
                    
                    # Convert the raw NMEA strings into floating point coordinates
                    lati = get_latitude(lat, lat_dir)
                    long = get_longitude(lon, lon_dir)
                    
                    # Break the loop and return the clean coordinates!
                    return lati, long
                else:
                    # Print how many satellites it currently sees!
                    sats = parts[7] if len(parts) > 7 else "0"
                    print(f"Waiting for lock... Satellites in view: {sats}")


# Displays IMU sensor information - Used Primarily for Testing Purposes
def imu_stuff():
  '''
  print("Temperature: {} degrees C".format(sensor.temperature))
  print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
  print("Magnetometer (microteslas): {}".format(sensor.magnetic))
  print("Gyroscope (rad/sec): {}".format(sensor.gyro))
  print("Euler angle: {}".format(sensor.euler))
  print("Quaternion: {}".format(sensor.quaternion))
  print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
  print("Gravity (m/s^2): {}".format(sensor.gravity))
  print()
  time.sleep(1)
  '''

def dataReceive(poll_obj):

    # Flash 3 times fast to signal "I am Alive!"
    for _ in range(3):
        led.value(1) # On
        time.sleep(0.1)
        led.value(0) # Off
        time.sleep(0.1)

    # Turn LED on solid to show we are entering the listening mode
    led.value(0)
    
    print("listening...")
    
    while True:
        if poll_obj.poll(0):
            value = input().strip()
            # Sometimes Windows sends an extra (or missing) newline - ignore them
            if value == "":
                continue
            print("Raw Input: {}".format(value))
            break

    # Temporary Geofence For testing purposes - Use computer-end-code.py for real Geofence Values
    #value = "OUTER, 40.39123253146508, -86.82833099365233, 40.365601883498044, -86.97458648681639, 40.42417189314546, -87.00067901611327, 40.468065962237894, -86.85236358642577, 40.428353515662465, -86.79743194580077, INNER"

    coordinateList = value.split(", ")
    innerBegin = coordinateList.index("INNER")

    outerList = [coordinateList[idx] for idx in range(1, innerBegin)]
    innerList = [coordinateList[idx] for idx in range(innerBegin + 1, len(coordinateList))]

    outerList = [float(i) for i in outerList]
    outerList = [(outerList[i], outerList[i+1]) for i in range(0, len(outerList)-1,2)] # Groups the latitudes and longitudes together

    innerList = [float(i) for i in innerList]
    innerList = [(innerList[i], innerList[i+1]) for i in range(0, len(innerList)-1,2)] # Groups the latitudes and longitudes together

    return outerList, innerList


def imu_update(latAvg, longAvg, time_interval, startTime,velocity_x,velocity_y):
    earth_radius = 6371000  # Earth's radius in meters

    time_elapsed = time.ticks_diff(time.ticks_ms(), startTime) / 1000.0

    imu_acceleration_x, imu_acceleration_y, imu_acceleration_z = sensor.lin_acc()

    #print("Accelerations")
    print(f"Acceleration X: {imu_acceleration_x:.10f}   Acceleration Y: {imu_acceleration_y:.10f}")
    #print("Sensor Linear Accelearion")
    #print(sensor.linear_acceleration)

    heading_rad = math.radians(sensor.euler()[0])  # yaw angle
    accel_north = imu_acceleration_x * math.cos(heading_rad) - imu_acceleration_y * math.sin(heading_rad)
    accel_east  = imu_acceleration_x * math.sin(heading_rad) + imu_acceleration_y * math.cos(heading_rad)

    # Velocity Estimation
    velocity_x += accel_north * time_elapsed
    velocity_y += accel_east * time_elapsed

    #print("VELOCITIES")
    #print(f"Velocity X: {velocity_x:.10f}   Velocity Y: {velocity_y:.10f}") 

    # Position Estimation
    latitude_change = ((velocity_x * time_elapsed) / earth_radius) * (180 / math.pi)
    longitude_change = ((velocity_y * time_elapsed) / earth_radius) * (180 / math.pi) / math.cos(math.radians(latAvg * (math.pi / 180)))
    
    #print("LAT AVG/LONGAVG")
    #print(f"Latitude: {latAvg:.10f}   Longitude: {longAvg:.10f}")

    #print("CHANGES")
    #print(f"Latitude Change: {latitude_change:.10f}   Longitude Change: {longitude_change:.10f}")  

    # Update latitude and longitude
    newlatAvg = latAvg + latitude_change
    newlongAvg = longAvg + longitude_change

    endTime = time.ticks_ms()
    print("IMU UPDATE")
    print(f"Latitude: {newlatAvg:.10f}   Longitude: {newlongAvg:.10f}")  
    #print("IMU Refresh Rate: ", float(endTime - startTime))

    return newlatAvg, newlongAvg, velocity_x, velocity_y
    

if __name__ == '__main__':
    led = Pin(25, Pin.OUT)
    poll_obj = select.poll()
    # Register sys.stdin (standard input) for monitoring read events with priority 1
    poll_obj.register(sys.stdin,1)
    
    while True:
        if poll_obj.poll(100): # Check every 100ms
            # Read everything currently in the buffer
            msg = sys.stdin.readline().strip()
            
            # Debug: see what we actually got
            # print("Received:", msg) 
            
            if "START" in msg:
                break
    
    outerPolygon, innerPolygon = dataReceive(poll_obj)
        
    # Flash 3 times fast to signal "I am Alive!"
    for _ in range(3):
        led.value(1) # On
        time.sleep(0.1)
        led.value(0) # Off
        time.sleep(0.1)

    # Turn LED on solid to show we are entering the listening mode
    led.value(0)

    # MicroPython I2C Initialization: GP15 (SCL) and GP14 (SDA) use I2C 1
    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
    
    # Initialize the IMU using the MicroPython driver we added to the /lib folder
    sensor = bno055.BNO055(i2c)

    last_val = 0xFFFF

    gps_uart = initialize_gps()                                     # Initializes GPS
    lcd_uart = initialize_lcd(backlight_red=255, backlight_green=1, backlight_blue=255)
    
    #lcd_uart.write(b"Connecting to GPS...            ")  # For 16x2 LCD
    #time.sleep(1.5) - Can add back in to display message for readability on LCD screen. The GPS sensor needs a few seconds to connect usually anyways. 
    
    # Example polygon for testing
    
    '''
    outerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]
    
    innerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]
    '''
    
    # CHANGE IMU SETTINGS HERE
    imu_update_points = 10 # This value can be further optimized. If set to zero, there will be no IMU points (only GPS points). 
    imu_time_interval = 0.1 # This value can be further optimized. See IMU BNO055 documentation for minimum refresh rate.
    
    while True:
        startTime = time.ticks_ms()
        #imu_stuff() Displays IMU Stuff
        latitude_avg, longitude_avg = 0,0
        
        latitude_avg, longitude_avg = get_current_location(gps_uart)    # Gets location info
        endTime = time.ticks_ms()
        
        imu_velocity_x = 0
        imu_velocity_y = 0

        print("GPS POINT BEFORE IMU UPDATES")
        print(f"Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}")    # Prints Lat and Long Info
        print("GPS Refresh Rate: ", float(endTime - startTime))
        
        if is_within_polygon(outerPolygon, (float(latitude_avg), float(longitude_avg))) is True and is_within_polygon(innerPolygon, (float(latitude_avg), float(longitude_avg))) is False:
            lcd_uart.write(b"IN                              ")  # For 16x2 LCD
            print("IN")
        else:
            lcd_uart.write(b"OUT                             ")  # For 16x2 LCD
            print("OUT")

        for i in range(imu_update_points):
            startTime = time.ticks_ms()
            time.sleep(imu_time_interval)
            latitude_avg, longitude_avg, imu_velocity_x, imu_velocity_y = imu_update(latitude_avg, longitude_avg, imu_time_interval, startTime, imu_velocity_x, imu_velocity_y)

            if is_within_polygon(outerPolygon, (float(latitude_avg), float(longitude_avg))) is True and is_within_polygon(innerPolygon, (float(latitude_avg), float(longitude_avg))) is False:
                lcd_uart.write(b"IN                              ")  # For 16x2 LCD
                print("IN")
            else:
                lcd_uart.write(b"OUT                             ")  # For 16x2 LCD
                print("OUT")
                
            #print("TEMP PRINT STATEMENT")
            #print(f"Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}")
        
        startTime = time.ticks_ms()

        lcd_uart.write(b"EPICS EVEI                      ")  # For 16x2 LCD
        
        # FUTURE TEAM COULD FIX THIS
        ###### This would print the information to the LCD - right now this messes up the spacing on the LCD for IN and OUT messages
        #time.sleep(1.5)
        #lcd_uart.write(b"Current Location:               ")  # For 16x2 LCD
        #time.sleep(1.5)
        #lcd_uart.write(b"Lat:  ")  # For 16x2 LCD
        #lcd_uart.write(latitude_avg.to_bytes(10, "big"))
        #print(latitude_avg.to_bytes(10, "big"))
        #lcd_uart.write(b" N   ")
        #time.sleep(1.5)
        #lcd_uart.write(b"Long: " + bytes(longitude_avg) + b" W   ")  # For 16x2 LCD
        #time.sleep(1.5) 
        
        #lcd_uart.write(b'                ')  # Clear display
        
        endTime = time.ticks_ms()
        print("GPS POINT AFTER IMU UPDATES")
        print(f"Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}")    # Prints Lat and Long Info
        print("GPS Refresh Rate: ", float(endTime - startTime))
