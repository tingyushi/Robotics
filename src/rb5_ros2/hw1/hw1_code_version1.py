"""
This file include code that control the robot motors
"""


from megapi import MegaPi

# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right  - correct
MFL = 11    # port for motor front left  - correct


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   

    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,-vbl)
        self.bot.motorRun(self.mbr,vbr)

    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)
    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()

    
    def carStraightForward(self, speeds):
        if self.verbose:
            print("CAR STRAIGHT FORWARD:")
        self.setFourMotors(speeds[0], speeds[1], speeds[2], speeds[3])


    def carStraightBackward(self, speeds):
        if self.verbose:
            print("CAR STRAIGHT BACKWARD:")
        self.setFourMotors(-speeds[0], -speeds[1], -speeds[2], -speeds[3])


    def carRotateClockwise(self, speeds):
        if self.verbose:
            print("CAR ROTATE CLOCKWISE:")
        self.setFourMotors(speeds[0], -speeds[1], speeds[2], -speeds[3])


    def carRotateCounterClockwise(self, speeds):
        if self.verbose:
            print("CAR ROTATE COUNTERCLOCKWISE:")
        self.setFourMotors(-speeds[0], speeds[1], -speeds[2], speeds[3])



    def carSlideRight(self, speeds):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speeds[0], -speeds[1], -speeds[2], speeds[3])
    
    
    def carSlideLeft(self, speeds):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(-speeds[0], speeds[1], speeds[2], -speeds[3])


    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()


def get_movement_info(x0, y0, theta0, x1, y1, theta1, rotation_method, no_rotation_range, scaling_factor):
    
    assert rotation_method in ["always_clockwise", "always_counterclockwise", "smallest", "biggest"]

    dx = x1 - x0
    dy = y1 - y0

    distance = math.sqrt((dx ** 2) + (dy ** 2)) * scaling_factor

    target_angle = math.atan2(dy, dx)
    
    first_rotation_angle, first_rotation_direction = angle_process(theta0, target_angle, rotation_method, no_rotation_range)
    assert 0<= first_rotation_angle <= 2 * math.pi

    second_rotation_angle, second_rotation_direction = angle_process(target_angle, theta1, rotation_method, no_rotation_range)
    assert 0<= second_rotation_angle <= 2 * math.pi

    return distance, first_rotation_angle, first_rotation_direction, second_rotation_angle, second_rotation_direction



def angle_process(start_angle, end_angle, rotation_method, no_rotation_range):

    angle_diff = end_angle - start_angle
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # normalize angle difference to [-pi, pi]

    if rotation_method == 'always_clockwise':
        if (-no_rotation_range <= angle_diff <= no_rotation_range):
            rotate_angle = angle_diff
            direction = "no"
        elif angle_diff > no_rotation_range:
            rotate_angle = 2 * math.pi - angle_diff
            direction = 'clockwise'
        else:
            rotate_angle = -angle_diff  
            direction = 'clockwise'
    elif rotation_method == 'always_counterclockwise':
        if (-no_rotation_range <= angle_diff <= no_rotation_range):
            rotate_angle = angle_diff
            direction = "no"
        elif angle_diff < -no_rotation_range:
            rotate_angle = 2 * math.pi + angle_diff
            direction = 'counterclockwise'
        else:
            rotate_angle = angle_diff
            direction = 'counterclockwise'
    elif rotation_method == 'smallest':
        if (-no_rotation_range <= angle_diff <= no_rotation_range):
            rotate_angle = angle_diff
            direction = 'no'
        elif angle_diff < -no_rotation_range:
            rotate_angle = -angle_diff
            direction = 'clockwise'
        else:
            rotate_angle = angle_diff
            direction = 'counterclockwise'
    elif rotation_method == "biggest":
        if (-no_rotation_range <= angle_diff <= no_rotation_range):
            rotate_angle = angle_diff
            direction = 'no'
        elif angle_diff < -no_rotation_range:
            rotate_angle = -angle_diff
            rotate_angle = 2 * math.pi - rotate_angle
            direction = 'counterclockwise'
        else:
            rotate_angle = angle_diff
            rotate_angle = 2 * math.pi - rotate_angle
            direction = 'clockwise'
    else:
        raise ValueError("Invalid rotation method.")
    
    return rotate_angle, direction




if __name__ == "__main__":
    import time
    import math
    
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(10)

    speeds = {"forward": [80, 72, 80, 72], "backward": [80, 72, 80, 72], "clockwise": [80, 72, 80, 72], "counterclockwise": [80, 72, 80, 72]}

    velocities = {"forward": 0.3, "backward": 0.3, "clockwise": 2.06, "counterclockwise": 2.026}
    
    waypoints = [(0, 0, 0), 
                 (-1, 0, 0), 
                 (-1, 1, math.pi/2), 
                 (-2, 1, 0), 
                 (-2, 2, -math.pi/2), 
                 (-1, 1, -math.pi/4), 
                 (0, 0, 0)]

    no_rotation_range = (10/180) * math.pi

    scaling_factor = 1

    rotation_method = "smallest"

    for (x0, y0, theta0), (x1, y1, theta1) in zip(waypoints[:-1], waypoints[1:]):
        
        print("------------------------------------------------------------------")
        print(f"Initial State: {(x0, y0, theta0)}")
        print(f"Final State: {(x1, y1, theta1)}")

        distance, first_rotation_angle, first_rotation_direction, second_rotation_angle, second_rotation_direction = get_movement_info(x0, y0, theta0, x1, y1, theta1, rotation_method, no_rotation_range, scaling_factor)
        
        print(f"{first_rotation_direction}: {first_rotation_angle/math.pi} pi --> Move: {distance} --> {second_rotation_direction}: {second_rotation_angle/math.pi} pi")

        print("------------------------------------------------------------------")

        # handle first rotation
        if first_rotation_direction == "clockwise":
            exe_time = first_rotation_angle / velocities['clockwise']
            mpi_ctrl.carRotateClockwise(speeds['clockwise'])
            print(exe_time)
            time.sleep(exe_time)
        elif first_rotation_direction == "counterclockwise":
            exe_time = first_rotation_angle / velocities['counterclockwise']
            mpi_ctrl.carRotateCounterClockwise(speeds['counterclockwise'])
            time.sleep(exe_time)
        elif first_rotation_direction == "no":
            pass
        else:
            assert False
        mpi_ctrl.carStop()
        time.sleep(1)


        # move forward
        exe_time = distance / velocities['forward']
        mpi_ctrl.carStraightForward(speeds['forward'])
        time.sleep(exe_time)
        mpi_ctrl.carStop()
        time.sleep(1)

        # handle second rotation
        if second_rotation_direction == "clockwise":
            exe_time = second_rotation_angle / velocities['clockwise']
            mpi_ctrl.carRotateClockwise(speeds['clockwise'])
            time.sleep(exe_time)
        elif second_rotation_direction == "counterclockwise":
            exe_time = second_rotation_angle / velocities['counterclockwise']
            mpi_ctrl.carRotateCounterClockwise(speeds['counterclockwise'])
            time.sleep(exe_time)
        elif second_rotation_direction == "no":
            pass
        else:
            assert False
        mpi_ctrl.carStop()
        time.sleep(3)


    mpi_ctrl.close()