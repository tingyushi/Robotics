import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
from threading import Thread
from mpi_controller_class import MegaPiController


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.target = None

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.target = np.array([targetx, targety, targetw])

    def calError(self, current_pos):
        result = self.target - current_pos
        angle_error = result[-1]
        result = np.abs(result)
        result[-1] = angle_error

        if abs(result[2]) > np.pi:
            if result[2] > 0:
                result[2] = (result[2] - 2 * np.pi)
            else:
                result[2] = (result[2] + 2 * np.pi)
        result[2] *= 0.5
        return result

    def update(self, measured_value):
        error = self.calError(measured_value)
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        return output



class APDetection(Node):
    def __init__(self):
        super().__init__('apdection')

        # Create the subscription
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.listener_callback,
            10)
        self.subscription  
        
        self.ap_poss = {"1": (0.75, 1.75, -90), "2": (1, 0, -180), "5": (-0.16, -0.165, 45), "3": (0.15, 1.5, 0)}
        #(1.1, 0.59, -135)
        self.latest_msgs = {}
        for frame_id in self.ap_poss:
            self.latest_msgs[frame_id] = None

    def listener_callback(self, msg):
        # Update the latest pose whenever a new message arrives
        self.latest_msgs[msg.header.frame_id] = msg
    


def spin_apdection(apdection):
    rclpy.spin(apdection)


def normalize_angle(degrees):
    """Normalize an angle to the range [-180, 180] degrees."""
    normalized = (degrees + 180) % 360 - 180
    # To handle the special case when the result is -180 (we want +180 instead)
    if normalized == -180:
        return 180
    return normalized



def create_transform_matrix(x, y, theta):
    T = np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])
    return T


def invert_transform_2d(rotation_matrix, translation_vector):
    rotation_inverse = rotation_matrix.T
    translation_inverse = -rotation_inverse @ translation_vector
    inverse_transform = np.array([
        [rotation_inverse[0, 0], rotation_inverse[0, 1], translation_inverse[0]],
        [rotation_inverse[1, 0], rotation_inverse[1, 1], translation_inverse[1]],
        [0, 0, 1]
    ])
    return inverse_transform


def get_car_world_coordinate_mat(pose_info, ap_pos):

    position = pose_info.pose.position
    orientation = pose_info.pose.orientation
    r = R.from_quat( [orientation.x, orientation.y, orientation.z, orientation.w] )
    euler_angles = r.as_euler('xyz', degrees=True)
    pitch, raw_yam_deg, roll = euler_angles
    yam_deg = normalize_angle( raw_yam_deg - 90 )
    yam_rad = np.deg2rad(yam_deg)
    
    # april-tag's x in camera coordinate system
    ap_cam_x = position.x

    # april-tag's y in world coordinate system
    ap_cam_y = position.z

    T_tag2world = create_transform_matrix(ap_pos[0], ap_pos[1], np.deg2rad(ap_pos[2]))

    T_tag2cam = create_transform_matrix(ap_cam_x, ap_cam_y, yam_rad)

    T_cam2tag = invert_transform_2d(T_tag2cam[:2,:2], [T_tag2cam[0][2],T_tag2cam[1][2]])

    T_cam2world = np.matmul(T_tag2world, T_cam2tag)

    angle_rad = np.pi / 2 + np.deg2rad(ap_pos[2]) - yam_rad
    angle_deg = normalize_angle(np.rad2deg(angle_rad))
    angle_rad = np.deg2rad(angle_deg)


    return np.matmul(T_cam2world,[[0],[0],[1]])[0].item(), np.matmul(T_cam2world,[[0],[0],[1]])[1].item(), angle_deg, angle_rad, T_cam2world


def get_car_world_coordinate_trig(pose_info, ap_pos_ts):

    position = pose_info.pose.position
    orientation = pose_info.pose.orientation
    r = R.from_quat( [orientation.x, orientation.y, orientation.z, orientation.w] )
    euler_angles = r.as_euler('xyz', degrees=True)
    pitch, raw_yam_deg, roll = euler_angles
    yam_deg = normalize_angle( raw_yam_deg - 90 )
    yam_rad = np.deg2rad(yam_deg)
    

    # april-tag's omega in camera coordinate system
    absr = np.abs( yam_deg )

    # april-tag's omega in world coordinate system
    abso = np.abs( normalize_angle( ap_pos_ts[2] ) )

    # april-tag's x in camera coordinate system
    ap_cam_x = position.x

    # april-tag's y in world coordinate system
    ap_cam_y = position.z

    # calculate car's omega in world coordinate system
    if 0 <= normalize_angle( ap_pos_ts[2] ) < 90:
        if (absr + abso <= 90):
            car_w_omega_deg = 90 + absr + abso
        else:
            car_w_omega_deg = -270 + absr + abso

    elif 90 <= normalize_angle( ap_pos_ts[2] ) <= 180:
        car_w_omega_deg = -270 + absr + abso
    
    elif -90 < normalize_angle( ap_pos_ts[2] ) < 0:
        if (absr - abso >= 90):
            car_w_omega_deg = -270 + absr - abso
        else:
            car_w_omega_deg = 90 + absr - abso
    
    elif -180 < normalize_angle( ap_pos_ts[2] ) <= -90:
        car_w_omega_deg = 90 + absr - abso
    
    else:
        assert False

    car_w_omega_deg = normalize_angle(car_w_omega_deg)
    car_w_omega_rad = np.deg2rad(car_w_omega_deg)
    

    # calculate car's x and y in world coordinate system
    distance = np.sqrt((ap_cam_x ** 2) + (ap_cam_y ** 2))
    theta = car_w_omega_deg
    
    if 0 <= theta < 90:
        absomega = np.abs( normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
        abstheta = np.abs(theta)

        if ap_cam_x < 0:
            if absomega + abstheta < 90:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( absomega + abstheta ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( absomega + abstheta ))
            else:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( 180 - absomega - abstheta ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( 180 - absomega - abstheta ))
        else:
            if absomega < abstheta:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( abstheta - absomega ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( abstheta - absomega ))
            else:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( absomega - abstheta ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( absomega - abstheta ))
    
    elif 90 <= theta <= 180:
        absomega = np.abs( normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
        abstheta = np.abs(theta)

        if ap_cam_x < 0:
            if absomega + abstheta < 180:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad(180 - absomega - abstheta ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad(180 - absomega - abstheta ))
            else:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( absomega + abstheta - 180 ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( absomega + abstheta - 180 ))
        else:
            if abstheta - absomega > 90:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( 180 - abstheta + absomega ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( 180 - abstheta + absomega ))
            else:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( abstheta - absomega ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( abstheta - absomega ))
    
    elif -90 < theta < 0:
        absomega = np.abs( normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
        abstheta = np.abs(theta)

        if ap_cam_x < 0:
            if absomega > abstheta:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( absomega - abstheta ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( absomega - abstheta ))
            else:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( abstheta - absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( abstheta - absomega ))
        else:
            if abstheta + absomega < 90:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( abstheta + absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( abstheta + absomega ))
            else:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( 180 - abstheta - absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( 180 - abstheta - absomega ))
    
    elif -180 < theta <= -90:
        absomega = np.abs( normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
        abstheta = np.abs(theta)

        if ap_cam_x < 0:
            if abstheta - absomega < 90:
                x = ap_pos_ts[0] - distance * np.cos(np.deg2rad( abstheta - absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( abstheta - absomega ))
            else:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( 180 - abstheta + absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( 180 - abstheta + absomega ))
        else:
            if abstheta + absomega < 180:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( 180 - abstheta - absomega ))
                y = ap_pos_ts[1] + distance * np.sin(np.deg2rad( 180 - abstheta - absomega ))
            else:
                x = ap_pos_ts[0] + distance * np.cos(np.deg2rad( abstheta + absomega - 180 ))
                y = ap_pos_ts[1] - distance * np.sin(np.deg2rad( abstheta + absomega - 180 ))
    
    else:
        assert False

    return x, y, car_w_omega_deg, car_w_omega_rad




def main(args=None):
    rclpy.init(args=args)

    way_points = np.array([ [0, 0, 0], 
    [0.75, 0, 0], 
    [0.75, 0, np.pi/2], 
    [0.75, 1.5, np.pi/2], 
    [0.75, 1.5, np.pi], 
    [0.75, 1.5, np.deg2rad(-1 * (np.rad2deg(np.arcsin(1 / np.sqrt(5))) + 90))], 
    [0, 0, np.deg2rad(-1 * (np.rad2deg(np.arcsin(1 / np.sqrt(5))) + 90))], 
    [0, 0, 0]]) 

    # Create and spin the subscriber node
    apdection = APDetection()
    # rclpy.spin(apdection)
    spin_thread = Thread(target=spin_apdection, args=(apdection,))
    spin_thread.start()

    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=False)  
    time.sleep(5)

    # pid = PID(kp=0.5, ki=0.05, kd=0.05, dt=1.0)
    
    pid = PID(kp=1, ki=0, kd=0, dt=1.0)

    straight_error_thresh = 0.1
    rotate_error_thresh = 0.15
    long_distance_thresh = 0.5

    max_runtime = 0.5

    unitspeed = 0.2

    pos_detected = False

    track = []

    # detect an initial frame id
    for frame_id in apdection.latest_msgs:
        if apdection.latest_msgs[frame_id] is not None:
            pos_detected = True
            detected_frame_id = frame_id
            detected_ap_pos = apdection.ap_poss[frame_id]
            detected_msg = apdection.latest_msgs[frame_id]
            break

    for way_point_id, (x, y, omega) in enumerate(way_points):
        pid.setTarget(x, y, omega)
        
        if way_point_id == 0:
            current_pos = way_points[way_point_id]
            continue
        
        previous_way_point = way_points[way_point_id - 1]
        current_way_point = way_points[way_point_id]

        if (previous_way_point[0] == current_way_point[0]) and (previous_way_point[1] == current_way_point[1]) and (previous_way_point[2] != current_way_point[2]):
            error_thresh = rotate_error_thresh
            rotating = True
        else:
            error_thresh = straight_error_thresh
            rotating = False

        if  way_point_id == 5:
            error_thresh = long_distance_thresh

        while(np.linalg.norm(pid.calError(current_pos)) > error_thresh):
            
            # get next move destination
            pid_value = pid.update(current_pos)
            print(f"pid_value x: {pid_value[0]}")
            print(f"pid_value y: {pid_value[1]}")
            print(f"pid_value w: {np.rad2deg(pid_value[2])}")
            print(f"target: {way_points[way_point_id]}")
            print(f"error_thresh: {error_thresh}")

            # calculate corresponding speed in 3 directions
            pid_norm = np.linalg.norm(pid_value)
            
            v_x_temp = (pid_value[0] / pid_norm) * unitspeed
            v_y_temp = (pid_value[1] / pid_norm) * unitspeed
            w_z = (pid_value[2] / pid_norm) * unitspeed

            if rotating:
                v_x = 0
                v_y = 0
                w_z = w_z
            else:
                absv_x = np.abs(v_x_temp)
                absv_y = np.abs(v_y_temp)
                absw_z = np.abs(w_z)
                if absv_x > absv_y:
                    v_x = absv_x
                    if detected_msg.pose.position.x < 0:
                        v_y = absv_y
                        w_z = absw_z
                    else:
                        v_y = -1 * absv_y
                        w_z = -1 * absw_z
                else:
                    v_x = absv_y
                    if detected_msg.pose.position.x < 0:
                        v_y = absv_x
                        w_z = absw_z
                    else:
                        v_y = -1 * absv_x
                        w_z = -1 * absw_z

            print(f"v_x: {v_x}")
            print(f"v_y: {v_y}")
            print(f"w_z: {w_z}")

            # move the car
            mpi_ctrl.carMixed(v_x, v_y, w_z)
            
            # move the car for 2 second
            runtime = pid_norm / unitspeed
            if runtime > max_runtime:
                runtime = max_runtime
            time.sleep(runtime)

            print(f"runtime: {runtime}")

            # stop to car
            mpi_ctrl.carStop()
            
            # wait for ap detection
            time.sleep(2)

            # check if we can see and apriltag
            pos_detected = False
            for frame_id in apdection.latest_msgs:
                if apdection.latest_msgs[frame_id] is not None:
                    pos_detected = True
                    detected_frame_id = frame_id
                    detected_ap_pos = apdection.ap_poss[frame_id]
                    detected_msg = apdection.latest_msgs[frame_id]
                    break
            
                
            # update current position
            if pos_detected:
                car_w_x, car_w_y, car_w_omega_deg, car_w_omega_rad = get_car_world_coordinate_trig(detected_msg, detected_ap_pos)
                current_pos = np.array([car_w_x, car_w_y, car_w_omega_rad])
            else:
                current_pos += pid_value
            
            print(f"current_pos_x: {car_w_x}")
            print(f"current_pos_y: {car_w_y}")
            print(f"current_pos_omega_deg: {car_w_omega_deg}")
            print(f"error norm: {np.linalg.norm(pid.calError(current_pos))}")
            print(f"error: {pid.calError(current_pos)}")
            print(f"detected_frame_id: {detected_frame_id}")
            print(); print()

            # record path
            track.append([car_w_x, car_w_y, car_w_omega_deg, car_w_omega_rad])

            # delete all the msg
            for frame_id in apdection.latest_msgs:
                apdection.latest_msgs[frame_id] = None
    

    # save track info
    track = np.array(track)
    np.savetxt("tract.txt", track, delimiter=",") 



def static_test(args=None):
    rclpy.init(args=args)

    # Create and spin the subscriber node
    apdection = APDetection()
    # rclpy.spin(apdection)
    spin_thread = Thread(target=spin_apdection, args=(apdection,))
    spin_thread.start()


    while True:

        for frame_id in apdection.latest_msgs:
            if apdection.latest_msgs[frame_id] is None:
                print(f'No pose received from frame_id: {frame_id}')
            else:
                car_w_x_trig, car_w_y_trig, car_w_omega_deg_trig, car_w_omega_rad_trig = get_car_world_coordinate_trig(apdection.latest_msgs[frame_id], apdection.ap_poss[frame_id])
                car_w_x_mat, car_w_y_mat, car_w_omega_deg_mat, car_w_omega_rad_mat, _ = get_car_world_coordinate_mat(apdection.latest_msgs[frame_id], apdection.ap_poss[frame_id])

                print(f"================ Using frame_id {frame_id} ==========================")
                print(f"car_w_omega_rad_trig: {car_w_omega_rad_trig}")
                print(f"car_w_omega_deg_trig: {car_w_omega_deg_trig}")
                print(f"car_w_x_trig: {car_w_x_trig}")
                print(f"car_w_y_trig: {car_w_y_trig}")
                print("-------------------------------------------------")
                print(f"car_w_omega_rad_mat: {car_w_omega_rad_mat}")
                print(f"car_w_omega_deg_mat: {car_w_omega_deg_mat}")
                print(f"car_w_x_mat: {car_w_x_mat}")
                print(f"car_w_y_mat: {car_w_y_mat}")

                apdection.latest_msgs[frame_id] = None

        time.sleep(1)
        print(); print(); print()


    # Cleanup when done
    apdection.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
    # static_test()