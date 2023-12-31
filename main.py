import pyrebase
import dronekit
import time
import ekf
import smc
import rbf
import numpy as np
from pymavlink import mavutil
import matplotlib.pyplot as plt
import pandas as pd
import os
import math

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
# vehicle = dronekit.connect("udp:127.0.0.1:14551", baud=115200, wait_ready=False, timeout=60)
home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
db.child("app").child("copters").child("0").child("home").child("latitude").set(home_lat)
db.child("app").child("copters").child("0").child("home").child("longitude").set(home_lon)

print("Success Connected")

def arm_and_takeoff(target_altitude, sliding=False):
    vehicle.mode = dronekit.VehicleMode("STABILIZE")
    db.child("app").child("copters").child("0").child("commands").child("mode").set("STABILIZE")
    # vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(1)
    # print("Prearm Check")
    # while not vehicle.is_armable:
    #     print("Waiting for ready")
    #     time.sleep(1)
    
    target = float(target_altitude)
    
    print("Taking Off")
    
    if not sliding:
        print("Arming motor")
        vehicle.mode = dronekit.VehicleMode("GUIDED")
        db.child("app").child("copters").child("0").child("commands").child("mode").set("GUIDED")    
        vehicle.armed = True
        # while not vehicle.armed:
        #     print("Waiting for arming")
        #     time.sleep(1)
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
    else:
        vehicle.mode = dronekit.VehicleMode("ACRO")
        print("Arming motor")
        db.child("app").child("copters").child("0").child("commands").child("mode").set("ACRO")    
        vehicle.armed = True
        # while not vehicle.armed:
        #     print("Waiting for arming")
        #     time.sleep(1)
        
        last_error_roll = 0
        last_error_pitch = 0
        last_error_yaw = 0
        last_error_alt = 0
        lambdaRef = db.child("app").child("copters").child("0").child("tuning").child("lambda").get().val()
        lambda_ = float(lambdaRef)
        deltaRef = db.child("app").child("copters").child("0").child("tuning").child("delta").get().val()
        delta_ = float(deltaRef)
        kDRef = db.child("app").child("copters").child("0").child("tuning").child("kd").get().val()
        kD_ = float(kDRef)
        while True:
            sliding_takeoff = smc.SlidingModeControl(lambda_, delta_, kD_)
            error_roll = 0 - vehicle.attitude.roll
            error_pitch = 0 - vehicle.attitude.pitch
            error_yaw = 0 - vehicle.attitude.yaw
            error_alt = target_altitude - vehicle.location.global_relative_frame.alt
            derivative_error_roll = error_roll - last_error_roll
            derivative_error_pitch = error_pitch - last_error_pitch
            derivative_error_yaw = error_yaw - last_error_yaw
            derivative_error_alt = error_alt - last_error_alt
            control_pitch = sliding_takeoff.takeoff(error_pitch, derivative_error_pitch)
            control_roll = sliding_takeoff.takeoff(error_roll, derivative_error_roll)
            control_yaw = sliding_takeoff.takeoff(error_yaw, derivative_error_yaw)
            control_alt = sliding_takeoff.takeoff(error_alt, derivative_error_alt)
            
            vehicle.channels.overrides = {
                '1': 1500 + int(control_roll),
                '2': 1500 + int(control_pitch),
                '3': 1500 + int(control_alt),
                '4': 1500 + int(control_yaw)
            }
            
            last_error_pitch = error_pitch
            last_error_yaw = error_yaw
            last_error_alt = error_alt
            
            db.child("app").child("copters").child("0").child("slide_mode").child("roll_control").set(control_roll)
            db.child("app").child("copters").child("0").child("slide_mode").child("roll_error").set(error_roll)
            db.child("app").child("copters").child("0").child("slide_mode").child("pitch_control").set(control_pitch)
            db.child("app").child("copters").child("0").child("slide_mode").child("pitch_error").set(error_pitch)
            db.child("app").child("copters").child("0").child("slide_mode").child("yaw_control").set(control_yaw)
            db.child("app").child("copters").child("0").child("slide_mode").child("yaw_error").set(error_yaw)
            db.child("app").child("copters").child("0").child("slide_mode").child("alt_control").set(control_alt)
            db.child("app").child("copters").child("0").child("slide_mode").child("alt_error").set(error_alt)
            
            if vehicle.location.global_relative_frame.alt >= target * 0.8:
                print("Reached target altitude")
                db.child("app").child("copters").child("0").child("commands").child("response").set("Reached Altitude.")
                time.sleep(1)
                db.child("app").child("copters").child("0").child("commands").child("action").set("disarm")
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
    print(f"Location: {vehicle.location.global_relative_frame}")
    return vehicle.location.global_relative_frame

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
        battery_percent = vehicle.battery.voltage
        db.child("app").child("copters").child("0").child("power").set(battery_percent)
        print(f"Battery updated: {battery_percent}%")
    else:
        print("Cannot get battery info.")
        
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def firebase_listener():
    # update_location_to_firebase()
    # update_battery_to_firebase()
    
    action = db.child("app").child("copters").child("0").child("commands").child("action").get().val()
    takeoff_alt = db.child("app").child("copters").child("0").child("commands").child("takeoff_alt").get().val()
    time_load = db.child("app").child("copters").child("0").child("commands").child("load_time").get().val()
    target_lat = db.child("app").child("copters").child("0").child("target").child("latitude").get().val()
    target_lon = db.child("app").child("copters").child("0").child("target").child("longitude").get().val()
    target_alt = db.child("app").child("copters").child("0").child("target").child("altitude").get().val()
    
    print(action)
    if action == "takeoff":
        arm_and_takeoff(takeoff_alt, False)
        time.sleep(2)
        # db.child("app").child("copters").child("0").child("commands").child("action").set("hover")
   
    elif action == "hover":
        print("Hovering...")
        
    elif action == "lift_load":
        print("Toggle Payload...")
        set_servo(9, 1100)
        set_servo(10, 1000)
        set_servo(11, 700)
        time.sleep(int(time_load))
        db.child("app").child("copters").child("0").child("commands").child("response").set("Lift Payload Success.")
        db.child("app").child("copters").child("0").child("commands").child("action").set("rtl")
        dronekit.VehicleMode("RTL")
        
    elif action == "go_to_target":
        print("Going to target coordinates...")
        target_location = dronekit.LocationGlobalRelative(float(target_lat), float(target_lon), float(target_alt))
        vehicle.simple_goto(target_location)
        db.child("app").child("copters").child("0").child("commands").child("response").set("Going to target.")
        while not has_arrived(target_location):
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            distance_to_target = haversine(current_lat, current_lon, float(target_lat), float(target_lon))
            print(f"Distance to target: {distance_to_target} km")
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
    
    elif action == "rtl":
        # print("RTL")
        # vehicle.mode = dronekit.VehicleMode("RTL")
        # db.child("app").child("copters").child("0").child("commands").child("mode").set("RTL")
        print("Going to home coordinate...")
        target_location = dronekit.LocationGlobalRelative(float(home_lat), float(home_lon), float(0.5))
        vehicle.simple_goto(target_location)
        db.child("app").child("copters").child("0").child("commands").child("response").set("Go Home Now.")
        while not has_arrived(target_location):
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            distance_to_target = haversine(current_lat, current_lon, float(target_lat), float(target_lon))
            print(f"Distance to target: {distance_to_target} km")
            print("Enroute to target...")
            time.sleep(5)
        db.child("app").child("copters").child("0").child("commands").child("response").set("RETURNING.")
        db.child("app").child("copters").child("0").child("commands").child("action").set("")
        time.sleep(5)
        vehicle.mode = dronekit.VehicleMode("LAND")
        db.child("app").child("copters").child("0").child("commands").child("mode").set("LAND")
        
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
    lambdaRef = db.child("app").child("copters").child("0").child("tuning").child("lambda").get().val()
    lambda_ = float(lambdaRef)
    deltaRef = db.child("app").child("copters").child("0").child("tuning").child("delta").get().val()
    delta_ = float(deltaRef)
    kDRef = db.child("app").child("copters").child("0").child("tuning").child("kd").get().val()
    kD_ = float(kDRef)
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
    print(f"X Value: {x0}")
    extended_kf_ = ekf.ExtendedKalmanFilter(F, H, Q, R, P, x0)
    slide_mode_ = smc.SlidingModeControl(lambda_, delta_, kD_)
    radial_base_roll_ = rbf.RBFNN(num_centers=10)
    radial_base_pitch_ = rbf.RBFNN(num_centers=10)
    radial_base_yaw_ = rbf.RBFNN(num_centers=10)
    angular_velocity_prev = {"rollspeed": 0, "pitchspeed": 0, "yawspeed": 0}
    time_prev = time.time()
    
    while True:
        time_curr = time.time()
        dt = time_curr - time_prev
        location_curr = vehicle.location.global_relative_frame
        orientation_curr = vehicle.attitude
        velocity_curr = vehicle.velocity
        print(f"Location: {location_curr}")
        print(f"Orientation: {orientation_curr}")
        print(f"Velocity: {velocity_curr}")
        takeoff_alt = db.child("app").child("copters").child("0").child("commands").child("takeoff_alt").get().val()
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
            vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt,
            vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw,
            vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2],
            roll_rate, pitch_rate, yaw_rate,
            ax, ay, az,
            roll_accel, pitch_accel, yaw_accel
        ])
        
        extended_kf_.update(z)
        extended_kfx = extended_kf_.x
        z_est = extended_kf_.x[2] # Alt
        roll_est = extended_kf_.x[3]  # Roll
        pitch_est = extended_kf_.x[4]  # Pitch
        yaw_est = extended_kf_.x[5]  # Yaw
        
        disturbance_estimated_roll = radial_base_roll_.forward(orientation_curr.roll)
        disturbance_estimated_pitch = radial_base_pitch_.forward(orientation_curr.pitch)
        disturbance_estimated_yaw = radial_base_yaw_.forward(orientation_curr.yaw)
        
        # error_roll = 0 - extended_kf_.x[3]
        # error_pitch = 0 - extended_kf_.x[4]
        # error_yaw = 0 - extended_kf_.x[5]
        error_alt = takeoff_alt - extended_kf_.x[2]
        
        error_roll = 0 - orientation_curr.roll
        error_pitch = 0 - orientation_curr.pitch
        error_yaw = 0 - orientation_curr.yaw
        # error_alt = float(takeoff_alt - location_curr.alt)
        
        radial_base_roll_.adapt_weights(orientation_curr.roll, error_roll)
        radial_base_pitch_.adapt_weights(orientation_curr.pitch, error_pitch)
        radial_base_yaw_.adapt_weights(orientation_curr.yaw, error_yaw)
        
        control_input_roll = slide_mode_.control_law(error_roll) + disturbance_estimated_roll
        control_input_pitch = slide_mode_.control_law(error_pitch) + disturbance_estimated_pitch
        control_input_yaw = slide_mode_.control_law(error_yaw) + disturbance_estimated_yaw
        control_input_alt = slide_mode_.control_law(error_alt)
        
        db.child("app").child("copters").child("0").child("slide_mode").child("roll_control").set(control_input_roll)
        db.child("app").child("copters").child("0").child("slide_mode").child("roll_error").set(error_roll)
        db.child("app").child("copters").child("0").child("slide_mode").child("pitch_control").set(control_input_pitch)
        db.child("app").child("copters").child("0").child("slide_mode").child("pitch_error").set(error_pitch)
        db.child("app").child("copters").child("0").child("slide_mode").child("yaw_control").set(control_input_yaw)
        db.child("app").child("copters").child("0").child("slide_mode").child("yaw_error").set(error_yaw)
        db.child("app").child("copters").child("0").child("slide_mode").child("alt_control").set(control_input_alt)
        db.child("app").child("copters").child("0").child("slide_mode").child("alt_error").set(error_alt)
        
        vehicle.channels.overrides['1'] = 1500 + round(control_input_roll)
        vehicle.channels.overrides['2'] = 1500 + round(control_input_pitch)
        vehicle.channels.overrides['3'] = 1500 + round(control_input_alt)
        vehicle.channels.overrides['4'] = 1500 + round(control_input_yaw)
        
        if z is not None and extended_kfx is not None: 
            comparation_ekf_data_["Measured"] = np.hstack((comparation_ekf_data_["Measured"], z.reshape(-1, 1)))
            comparation_ekf_data_["Predicted"] = np.hstack((comparation_ekf_data_["Predicted"], extended_kfx.reshape(-1, 1)))
        
        print("Waiting for commands...")
        time.sleep(0.5)
        firebase_listener()
        time.sleep(0.5)
        orientation = orientation_curr
        velocity = velocity_curr
        time_prev = time_curr
        angular_velocity_prev = {"rollspeed": roll_rate, "pitchspeed": pitch_rate, "yawspeed": yaw_rate}
        if vehicle.mode.name == 'LAND':
            if vehicle.location.global_relative_frame.alt < 1:
                time.sleep(1)
                vehicle.channels.overrides = {}
                break
            
    iteration = int(db.child("app").child("copters").child("0").child("iteration").get().val())
    plt.figure(figsize=(25, 20))
    plt.suptitle("Hayago Extended Kalman Filter Prediction")
    for i in range(18):
        ax = plt.subplot(6, 3, i+1)
        ax.set_title(state_names[i])
        ax.plot(comparation_ekf_data_["Measured"][i, :], label="Measured")
        ax.plot(comparation_ekf_data_["Predicted"][i, :], label="Predicted")
    plt.legend()
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    filename = f"ekf_data_{iteration}"
    plt.savefig(filename+".png")
    errors = comparation_ekf_data_["Measured"] - comparation_ekf_data_["Predicted"]
    rmse = np.sqrt(np.mean(errors**2, axis=1))
    df_errors = pd.DataFrame(errors.transpose(), columns=[f"Error_{state}" for state in state_names])
    df_rmse = pd.DataFrame([rmse], columns=[f"RMSE_{state}" for state in state_names])  # Memperbaiki di sini
    df_measured = pd.DataFrame(comparation_ekf_data_["Measured"].transpose(), columns=[f"Measured_{state}" for state in state_names])
    df_predicted = pd.DataFrame(comparation_ekf_data_["Predicted"].transpose(), columns=[f"Predicted_{state}" for state in state_names])
    df_to_save = pd.concat([df_measured, df_predicted, df_rmse], axis=1)
    df_to_save.to_csv(filename+".csv", index=False)

    storage.child("drone/data/"+filename+".png").put(filename+".png")
    storage.child("drone/data/"+filename+".csv").put(filename+".csv")
    db.child("app").child("copters").child("0").child("iteration").set(iteration+1)
    os.remove(filename+".png")
    os.remove(filename+".csv")
    db.child("app").child("copters").child("0").child("commands").child("mode").set("ACRO")
else:
    print("Drone is not connected")
        
# except Exception as e:
#     print("An error occurred:", str(e))
#     vehicle.mode = dronekit.VehicleMode("RTL")
#     db.child("app").child("copters").child("0").child("commands").child("mode").set("RTL")
#     # end_drone()