import pyrebase
import dronekit
import time
import ekf
import numpy as np
from pymavlink import mavutil
import matplotlib.pyplot as plt
import pandas as pd
import uuid
import os

config = {
    "apiKey": "AIzaSyBi8dJvahsGnlEJxt2XW9CbCVCZ_F8QbIA",
    "authDomain": "eco-enzym.firebaseapp.com",
    "databaseURL": "https://eco-enzym-default-rtdb.asia-southeast1.firebasedatabase.app",
    "storageBucket": "eco-enzym.appspot.com",
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()
storage = firebase.storage()

comparation_ekf_data_ = {
    "Measured": np.empty((18, 0)),
    "Predicted": np.empty((18, 0))
}

    
state_names = [
    "x", "y", "z",
    "roll", "pitch", "yaw",
    "x'", "y'", "z'",
    "roll'", "pitch'", "yaw'",
    "x''", "y''", "z''",
    "roll''", "pitch''", "yaw''"
]

def end_drone():
    db.child("app").child("copters").child("0").child("actived").set(False)

vehicle = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=True, timeout=60)
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
    
def get_orientation_data():
    return vehicle.attitude

def get_velocity_data():
    return vehicle.velocity

def get_relative_location_data():
    return vehicle.location.global_relative_frame

def get_absolute_location_data():
    return vehicle.location.global_frame

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
        
# try:
db.child("app").child("copters").child("0").child("actived").set(True)
connected = db.child("app").child("copters").child("0").child("connected").get().val()
if connected:
    
    # INITIAL EKF
    x0 = np.zeros(18)
    dt = 0.1
    F = np.eye(18)
    F[0, 6] = F[1, 7] = F[2, 8] = F[3, 9] = F[4, 10] = F[5, 11] = dt
    F[6, 12] = F[7, 13] = F[8, 14] = F[9, 15] = F[10, 16] = F[11, 17] = dt
    H = np.eye(18)
    q_val = 0.01
    Q = np.diag([q_val] * 18)
    r_val = 0.1
    R = np.diag([r_val] * 18)
    p_val = 0.1
    P = np.diag([p_val] * 18)
    location = get_relative_location_data()
    orientation = get_orientation_data()
    velocity = get_velocity_data()
    x0 = np.array([location.lat, location.lon, location.alt,
                    orientation.roll, orientation.pitch, orientation.yaw,
                    velocity[0], velocity[1], velocity[2],
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0])
    extended_kf_ = ekf.ExtendedKalmanFilter(F, H, Q, R, P, x0)
    angular_velocity_prev = {"rollspeed": 0, "pitchspeed": 0, "yawspeed": 0}
    time_prev = time.time()
    
    while True:
        time_curr = time.time()
        dt = time_curr - time_prev
        location_curr = get_relative_location_data()
        orientation_curr = get_orientation_data()
        velocity_curr = get_velocity_data()
        # Coordinate Accel
        ax = (velocity_curr[0] - velocity[0]) / dt
        ay = (velocity_curr[1] - velocity[1]) / dt
        az = (velocity_curr[2] - velocity[2]) / dt
        # Orientation Speed
        roll_rate = (orientation_curr.roll - orientation.roll) / dt
        pitch_rate = (orientation_curr.pitch - orientation.pitch) / dt
        yaw_rate = (orientation_curr.yaw - orientation.yaw) / dt
        # Orientation Accel
        roll_accel = (roll_rate - angular_velocity_prev["rollspeed"]) / dt
        pitch_accel = (pitch_rate - angular_velocity_prev["pitchspeed"]) / dt
        yaw_accel = (yaw_rate - angular_velocity_prev["yawspeed"]) / dt
        
        extended_kf_.predict()
        z = np.array([
            vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt,
            vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw,
            vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2],
            roll_rate, pitch_rate, yaw_rate,
            ax, ay, az,
            roll_accel, pitch_accel, yaw_accel
        ])
        extended_kf_.update(z)
        extended_kfx = extended_kf_.x
        if z is not None and extended_kfx is not None: 
            comparation_ekf_data_["Measured"] = np.hstack((comparation_ekf_data_["Measured"], z.reshape(-1, 1)))
            comparation_ekf_data_["Predicted"] = np.hstack((comparation_ekf_data_["Predicted"], extended_kfx.reshape(-1, 1)))

        print("Waiting for commands...")
        time.sleep(0.5)
        firebase_listener()
        time.sleep(2)
        orientation = orientation_curr
        velocity = velocity_curr
        time_prev = time_curr
        angular_velocity_prev = {"rollspeed": roll_rate, "pitchspeed": pitch_rate, "yawspeed": yaw_rate}
        if vehicle.mode.name == 'LAND':
            if vehicle.location.global_relative_frame.alt < 1:
                time.sleep(1)
                break
    plt.figure(figsize=(25, 20))
    plt.suptitle("Hayago Extended Kalman Filter Prediction")
    for i in range(18):
        ax = plt.subplot(6, 3, i+1)
        ax.set_title(state_names[i])
        ax.plot(comparation_ekf_data_["Measured"][i, :], label="Measured")
        ax.plot(comparation_ekf_data_["Predicted"][i, :], label="Predicted")
    plt.legend()
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    filename = "ekf_data"
    plt.savefig(filename+".png")
    
    errors = comparation_ekf_data_["Measured"] - comparation_ekf_data_["Predicted"]
    rmse = np.sqrt(np.mean(errors**2, axis=1))
    headers = [f"Measured_{state}" for state in state_names] + [f"Predicted_{state}" for state in state_names] + [f"Error_RMS_{state}" for state in state_names]
    data_combined = np.hstack((comparation_ekf_data_["Measured"], comparation_ekf_data_["Predicted"], rmse.reshape(-1, 1)))
    df = pd.DataFrame(data_combined, columns=headers)
    df.to_csv(filename+".csv", index=False)
    
    storage.child("drone/data/"+filename+".png").put(filename+".png")
    storage.child("drone/data/"+filename+".csv").put(filename+".csv")
    os.remove(filename+".png")
    os.remove(filename+".csv")
    
else:
    print("Drone is not connected")
        
# except Exception as e:
#     print("An error occurred:", str(e))
#     vehicle.mode = dronekit.VehicleMode("RTL")
#     db.child("app").child("copters").child("0").child("commands").child("mode").set("RTL")
#     # end_drone()