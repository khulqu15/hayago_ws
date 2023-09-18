import dronekit
import time
from pymavlink import mavutil

print("[0] 57600")
print("[1] 115200")
baudrate = int(input("Choose baudrate: ") or 0)

force_connect = input("Force Connect? [Y/N] : ")
if force_connect.upper() == 'Y':
    is_force = False
else:
    is_force = True

baudrates = [
    57600,
    115200,
]

try:
    vehicle = dronekit.connect("/dev/ttyACM0", baud=baudrates[baudrate], wait_ready=is_force, timeout=60)
except dronekit.TimeoutError:
    print("Failed to connect after 60 seconds. Try forcing the connection or check the drone's status.")

def arm_and_takeoff(target_altitude):
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)
    
    print("Prearm Check")
    # while not vehicle.is_armable:
    #     print("Waiting for ready")
    #     time.sleep(1)
        
    print("Arming motor")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(1)
    
    print("Taking Off")
    vehicle.simple_takeoff(target_altitude)
    
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
        
def set_servo(number, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        number,
        pwm,
        0,0,0,0.0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
        
arm_and_takeoff(2)

print("Hovering for 10 seconds...")
time.sleep(5)

print("Lowering Payload")
set_servo(1, 1500)
print("Control servo aux")
set_servo(2, 1000)
set_servo(3, 2000)
time.sleep(1)

print("Landing")
vehicle.mode = dronekit.VehicleMode("LAND")
while vehicle.armed:
    print("Waiting for landing...")
    time.sleep(1)
print("Disarmed. Mission completed!")
vehicle.close()