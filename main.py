import pyrebase
import dronekit
import time
from pymavlink import mavutil

config = {
    "apiKey": "AIzaSyBi8dJvahsGnlEJxt2XW9CbCVCZ_F8QbIA",
    "authDomain": "eco-enzym.firebaseapp.com",
    "databaseURL": "https://eco-enzym-default-rtdb.asia-southeast1.firebasedatabase.app",
    "storageBucket": "eco-enzym.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

vehicle = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=False, timeout=60)
print("Success Connected")

def arm_and_takeoff(target_altitude):
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)
    
    # print("Prearm Check")
    # while not vehicle.is_armable:
    #     print("Waiting for ready")
    #     time.sleep(1)
        
    print("Arming motor")
    # vehicle.mode = dronekit.VehicleMode("STABILIZE")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True
    
    # while not vehicle.armed:
    #     print("Waiting for arming")
    #     time.sleep(1)
    
    print("Taking Off")
    vehicle.simple_takeoff(target_altitude)
    
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    db.child("app").child("copters").child("0").child("commands").child("action").set("hover")
    time.sleep(5)
    db.child("app").child("copters").child("0").child("commands").child("action").set("control_servos")
    time.sleep(2)
    db.child("app").child("copters").child("0").child("commands").child("action").set("lower_payload")
    time.sleep(5)
    db.child("app").child("copters").child("0").child("commands").child("action").set("land")
        
def set_servo(number, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        number,
        pwm,
        0,0,0,0,0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def firebase_listener():
    action = db.child("app").child("copters").child("0").child("commands").child("action").get().val()
    print(action)
    if action == "takeoff":
        arm_and_takeoff(1.5)
    elif action == "hover":
        print("Hovering for 10 seconds...")
        time.sleep(5)
    elif action == "lower_payload":
        print("Lowering payload on winch (katrol)...")
        set_servo(9, 1500)
    elif action == "control_servos":
        print("Controlling servos on AUX2 and AUX3...")
        set_servo(10, 1000)
        time.sleep(1)
        set_servo(11, 1000)
        time.sleep(1)
    elif action == "land":
        print("Landing...")
        vehicle.mode = dronekit.VehicleMode("LAND")
        db.child("app").child("copters").child("0").child("commands").child("action").set("")

# arm_and_takeoff(2)

# print("Hovering for 10 seconds...")
# time.sleep(5)

# print("Lowering Payload")
# set_servo(1, 1500)
# print("Control servo aux")
# set_servo(2, 1000)
# set_servo(3, 2000)
# time.sleep(1)

# print("Landing")
# vehicle.mode = dronekit.VehicleMode("LAND")
# while vehicle.armed:
#     print("Waiting for landing...")
#     time.sleep(1)
# print("Disarmed. Mission completed!")
# vehicle.close()

while True:
    print("Waiting for commands...")
    firebase_listener()
    time.sleep(5)