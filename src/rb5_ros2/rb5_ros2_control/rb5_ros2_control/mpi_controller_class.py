
from megapi import MegaPi
import numpy as np

# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
# MFR = 2     # port for motor front right
# MBL = 3     # port for motor back left
# MBR = 10    # port for motor back right  - correct
# MFL = 11    # port for motor front left  - correct


MFR = 3     # port for motor front right
MBL = 2     # port for motor back left
MBR = 11    # port for motor back right  - correct
MFL = 10    # port for motor front left  - correct


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
        
        self.r = 0.03
        self.lx = 0.055
        self.ly = 0.675

        self.jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)], [1, 1, (self.lx + self.ly)], [1, 1, -(self.lx + self.ly)], [1, -1, (self.lx + self.ly)]]) / self.r


    
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
            print("CAR SLIDE RIGHT:")
        self.setFourMotors(speeds[0], -speeds[1], -speeds[2], speeds[3])

    
    def carSlideLeft(self, speeds):
        if self.verbose:
            print("CAR SLIDE LEFT:")
        self.setFourMotors(-speeds[0], speeds[1], speeds[2], -speeds[3])


    def carMixed(self, v_x, v_y, w_z):
        
        wheel_angular_speeds = np.matmul(self.jacobian_matrix , np.reshape(np.array([v_x, v_y, w_z]), (-1, 1)))

        wheel_angular_speeds = wheel_angular_speeds.flatten()

        fl_motor = self.fl(wheel_angular_speeds[0])
        fr_motor = self.fr(wheel_angular_speeds[1])
        bl_motor = self.bl(wheel_angular_speeds[2])
        br_motor = self.br(wheel_angular_speeds[3])

        if self.verbose:
            print("CAR MIXED:")
        self.setFourMotors(fl_motor, fr_motor, bl_motor, br_motor)


    def close(self):
        self.bot.close()
        self.bot.exit()


    # given angular speed, return motor value
    def fl(self, angular_speed):
        if angular_speed > 0:
            mv = 4.771616806064096 * angular_speed + 21.832778192772544 
        else:
            mv = 4.786422933886705 * angular_speed - 21.767647504575823 
        return int(round(mv))
    

    # given angular speed, return motor value
    def fr(self, angular_speed):
        if angular_speed > 0:
            mv = 4.144512068184324 * angular_speed + 34.79747370225688 
        else:
            mv = 3.973617636429344 * angular_speed - 34.68142392482835 
        return int(round(mv))


    # given angular speed, return motor value
    def bl(self, angular_speed):
        if angular_speed > 0:
            mv = 5.189686631317275 * angular_speed + 17.856314519782998 
        else:
            mv = 4.888023596068874 * angular_speed - 20.77842545204427 
        return int(round(mv))

    
    # given angular speed, return motor value
    def br(self, angular_speed):
        if angular_speed > 0:
            mv = 4.771616806064096 * angular_speed + 28.832778192772544 
        else:
            mv =  4.786422933886705 * angular_speed - 26.767647504575823 
        return int(round(mv))