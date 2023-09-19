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

def end_drone():
    db.child("app").child("copters").child("0").child("actived").set(False)

vehicle = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=False, timeout=60)
print("Success Connected")

def arm_and_takeoff(target_altitude):
    vehicle.mode = dronekit.VehicleMode("STABILIZE")
    db.child("app").child("copters").child("0").child("commands").child("mode").set("STABILIZE")
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)
    
    # print("Prearm Check")
    # while not vehicle.is_armable:
    #     print("Waiting for ready")
    #     time.sleep(1)
        
    print("Arming motor")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    db.child("app").child("copters").child("0").child("commands").child("mode").set("GUIDED")    
    vehicle.armed = True
    
    # while not vehicle.armed:
    #     print("Waiting for arming")
    #     time.sleep(1)
    
    target = float(target_altitude)
    
    print("Taking Off")
    vehicle.simple_takeoff(target)
    
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target * 0.95:
            print("Reached target altitude")
            db.child("app").child("copters").child("0").child("commands").child("response").set("Reached Altitude.")
            time.sleep(1)
            db.child("app").child("copters").child("0").child("commands").child("action").set("go_to_target")
            break
        time.sleep(1)
        
def set_servo(number, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        1,
        number,
        pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def has_arrived(target_location, threshold=0.0001):
    current_location = vehicle.location.global_relative_frame
    lat_diff = abs(current_location.lat - target_location.lat)
    lon_diff = abs(current_location.lon - target_location.lon)
    return lat_diff < threshold and lon_diff < threshold
    
def update_location_to_firebase():
    latitude = vehicle.location.global_relative_frame.lat
    longitude = vehicle.location.global_relative_frame.lon
    altitude = vehicle.location.global_relative_frame.alt
    db.child("app").child("copters").child("0").child("latitude").set(latitude)
    db.child("app").child("copters").child("0").child("longitude").set(longitude)
    db.child("app").child("copters").child("0").child("altitude").set(altitude)
    print(f"Location updated: Lat={latitude}, Lon={longitude}, Alt={altitude}")

def update_battery_to_firebase():
    if vehicle.battery:
        battery_percent = vehicle.battery.level
        db.child("app").child("copters").child("0").child("power").set(battery_percent)
        print(f"Battery updated: {battery_percent}%")
    else:
        print("Cannot get battery info.")

def firebase_listener():
    update_location_to_firebase()
    update_battery_to_firebase()
    
    action = db.child("app").child("copters").child("0").child("commands").child("action").get().val()
    takeoff_alt = db.child("app").child("copters").child("0").child("commands").child("takeoff_alt").get().val()
    time_load = db.child("app").child("copters").child("0").child("commands").child("load_time").get().val()
    target_lat = db.child("app").child("copters").child("0").child("target").child("latitude").get().val()
    target_lon = db.child("app").child("copters").child("0").child("target").child("longitude").get().val()
    target_alt = db.child("app").child("copters").child("0").child("target").child("altitude").get().val()
    
    print(action)
    if action == "takeoff":
        arm_and_takeoff(takeoff_alt)
        time.sleep(2)
        db.child("app").child("copters").child("0").child("commands").child("action").set("hover")
   
    elif action == "hover":
        print("Hovering...")
        
    elif action == "lift_load":
        print("Toggle Payload...")
        set_servo(9, 1100)
        set_servo(10, 1000)
        set_servo(11, 700)
        time.sleep(int(time_load))
        db.child("app").child("copters").child("0").child("commands").child("response").set("Lift Payload Success.")
        dronekit.VehicleMode("RTL")
        
    elif action == "go_to_target":
        print("Going to target coordinates...")
        target_location = dronekit.LocationGlobalRelative(float(target_lat), float(target_lon), float(target_alt))
        vehicle.simple_goto(target_location)
        db.child("app").child("copters").child("0").child("commands").child("response").set("Going to target.")
        while not has_arrived(target_location):
            print("Enroute to target...")
            time.sleep(5)
        print("Arrived at target location!")
        db.child("app").child("copters").child("0").child("commands").child("action").set("unload")

    elif action == "unload":
        print("Toggle Payload...")
        set_servo(9, 1900)
        set_servo(10, 800)
        set_servo(11, 500)
        time.sleep(int(time_load))
        db.child("app").child("copters").child("0").child("commands").child("response").set("Unload Success.")
        db.child("app").child("copters").child("0").child("commands").child("action").set("lift_load")
        
    elif action == "disarm":
        print("Landing...")
        vehicle.mode = dronekit.VehicleMode("LAND")
        db.child("app").child("copters").child("0").child("commands").child("mode").set("LAND")
        db.child("app").child("copters").child("0").child("commands").child("response").set("Unload Success.")
        db.child("app").child("copters").child("0").child("commands").child("action").set("")
        
    mode = db.child("app").child("copters").child("0").child("commands").child("mode").get().val()
    print(f"Mode from Firebase: {mode}")
    if mode and mode != vehicle.mode.name:
        try:
            vehicle.mode = dronekit.VehicleMode(mode)
            print(f"Changed mode to {mode}")
        except Exception as e:
            print(f"Error changing mode: {e}")
        
try:
    db.child("app").child("copters").child("0").child("actived").set(True)
    connected = db.child("app").child("copters").child("0").child("connected").get().val()
    if connected:
        while True:
            print("Waiting for commands...")
            firebase_listener()
            time.sleep(2)
    else:
        print("Drone is not connected")
        
except Exception as e:
    print("An error occurred:", str(e))
    vehicle.mode = dronekit.VehicleMode("RTL")
    db.child("app").child("copters").child("0").child("commands").child("mode").set("RTL")
    # end_drone()
