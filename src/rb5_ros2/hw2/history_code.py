import numpy as np

def rotation_matrix(self, angle):
        """Create a 2D rotation matrix for a given angle in radians."""
        return np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])



def homogeneous_matrix(self, x, y, angle):
    """Create a 3x3 homogeneous transformation matrix."""
    R = self.rotation_matrix(angle)
    T = np.array([
        [R[0, 0], R[0, 1], x],
        [R[1, 0], R[1, 1], y],
        [0, 0, 1]
    ])
    return T


def get_car_world_coordinate(self):
    
    assert self.latest_pose is not None

    car_w_x = 0 
    car_w_y = 0
    car_w_omega_deg = 0

    # get april tag in camera_coordinate system
    tag_in_cam_position = self.latest_pose.pose.position
    orientation = self.latest_pose.pose.orientation
    r = R.from_quat( [orientation.x, orientation.y, orientation.z, orientation.w] )
    euler_angles = r.as_euler('xyz', degrees=True)
    pitch, yaw, roll = euler_angles
    yaw_rad = np.deg2rad(yaw)

    ap_cam_x = tag_in_cam_position.x
    ap_cam_y = tag_in_cam_position.z
    ap_cam_omega_deg = yam_deg
    
    print(f"ap_cam_x: {ap_cam_x}")
    print(f"ap_cam_y: {ap_cam_y}")
    # print(f"ap_cam_omega_deg: {ap_cam_omega_deg}")
    # print(f"ap_cam_omega_rad: {ap_cam_omega_rad}")

    T_world_tag = self.homogeneous_matrix(self.ap_pos[0], self.ap_pos[1], self.ap_pos[2])
    T_camera_tag = self.homogeneous_matrix(ap_cam_x, ap_cam_y, ap_cam_omega_rad)
    T_tag_camera = np.linalg.inv(T_camera_tag)
    T_world_camera = T_world_tag @ T_tag_camera

    car_w_x = T_world_camera[0, 2]
    car_w_y = T_world_camera[1, 2]
    omega_temp = np.arctan2(T_world_camera[1, 0], T_world_camera[0, 0])
    car_w_omega_deg = self.normalize_angle( np.rad2deg(omega_temp) + 90 )
    car_w_omega_rad = np.deg2rad(car_w_omega_deg)

    return car_w_x, car_w_y, car_w_omega_rad, car_w_omega_deg






    def create_transform_matrix(self, x, y, theta):
        T = np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])
        return T


    def invert_transform_2d(self, rotation_matrix, translation_vector):
        rotation_inverse = rotation_matrix.T
        translation_inverse = -rotation_inverse @ translation_vector
        inverse_transform = np.array([
            [rotation_inverse[0, 0], rotation_inverse[0, 1], translation_inverse[0]],
            [rotation_inverse[1, 0], rotation_inverse[1, 1], translation_inverse[1]],
            [0, 0, 1]
        ])
        return inverse_transform


    def get_car_world_coordinate_mat(self, pose_info, ap_pos):

        position = pose_info.pose.position
        orientation = pose_info.pose.orientation
        r = R.from_quat( [orientation.x, orientation.y, orientation.z, orientation.w] )
        euler_angles = r.as_euler('xyz', degrees=True)
        pitch, raw_yam_deg, roll = euler_angles
        yam_deg = self.normalize_angle( raw_yam_deg - 90 )
        yam_rad = np.deg2rad(yam_deg)
        
        # april-tag's x in camera coordinate system
        ap_cam_x = position.x

        # april-tag's y in world coordinate system
        ap_cam_y = position.z

        T_tag2world = self.create_transform_matrix(ap_pos[0], ap_pos[1], np.deg2rad(ap_pos[2]))

        T_tag2cam = self.create_transform_matrix(ap_cam_x, ap_cam_y, yam_rad)

        T_cam2tag = self.invert_transform_2d(T_tag2cam[:2,:2], [T_tag2cam[0][2],T_tag2cam[1][2]])

        T_cam2world = np.matmul(T_tag2world, T_cam2tag)

        angle_rad = np.pi / 2 + np.deg2rad(ap_pos[2]) - yam_rad
        angle_deg = np.rad2deg(np.pi / 2 + np.deg2rad(ap_pos[2]) - yam_rad)

        return np.matmul(T_cam2world,[[0],[0],[1]])[0].item(), np.matmul(T_cam2world,[[0],[0],[1]])[1].item(), angle_deg, angle_rad


    def get_car_world_coordinate_trig(self, pose_info, ap_pos_ts):

        position = pose_info.pose.position
        orientation = pose_info.pose.orientation
        r = R.from_quat( [orientation.x, orientation.y, orientation.z, orientation.w] )
        euler_angles = r.as_euler('xyz', degrees=True)
        pitch, raw_yam_deg, roll = euler_angles
        yam_deg = self.normalize_angle( raw_yam_deg - 90 )
        yam_rad = np.deg2rad(yam_deg)
        
        # april-tag's omega in camera coordinate system
        absr = np.abs( yam_deg )

        # april-tag's omega in world coordinate system
        abso = np.abs( self.normalize_angle( ap_pos_ts[2] ) )

        # april-tag's x in camera coordinate system
        ap_cam_x = position.x

        # april-tag's y in world coordinate system
        ap_cam_y = position.z

        # calculate car's omega in world coordinate system
        if 0 <= self.normalize_angle( ap_pos_ts[2] ) < 90:
            if (absr + abso <= 90):
                car_w_omega_deg = 90 + absr + abso
            else:
                car_w_omega_deg = -270 + absr + abso

        elif 90 <= self.normalize_angle( ap_pos_ts[2] ) <= 180:
            car_w_omega_deg = -270 + absr + abso
        
        elif -90 < self.normalize_angle( ap_pos_ts[2] ) < 0:
            if (absr - abso >= 90):
                car_w_omega_deg = -270 + absr - abso
            else:
                car_w_omega_deg = 90 + absr - abso
        
        elif -180 < self.normalize_angle( ap_pos_ts[2] ) <= -90:
            car_w_omega_deg = 90 + absr - abso
        
        else:
            assert False

        car_w_omega_deg = self.normalize_angle(car_w_omega_deg)
        car_w_omega_rad = np.deg2rad(car_w_omega_deg)
        

        # calculate car's x and y in world coordinate system
        distance = np.sqrt((ap_cam_x ** 2) + (ap_cam_y ** 2))
        theta = car_w_omega_deg
        
        if 0 <= theta < 90:
            absomega = np.abs( self.normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
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
            absomega = np.abs( self.normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
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
            absomega = np.abs( self.normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
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
            absomega = np.abs( self.normalize_angle( np.rad2deg( np.arctan( np.abs(ap_cam_x) / np.abs(ap_cam_y) ) ) ) )
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


def normalize_angle(self, degrees):
    """Normalize an angle to the range [-180, 180] degrees."""
    normalized = (degrees + 180) % 360 - 180
    # To handle the special case when the result is -180 (we want +180 instead)
    if normalized == -180:
        return 180
    return normalized