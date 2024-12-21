"""
This file include code that control the robot motors
"""


if __name__ == "__main__":
    import time 
    import sys ; sys.path.append("../")
    from mpi_controller_class import MegaPiController

    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(2)
    
    # forwardSpeeds = [71, 78, 71, 78]
    forwardSpeeds = [63, 70, 63, 70]

    mpi_ctrl.carStraightForward(forwardSpeeds)
    time.sleep(5)

    mpi_ctrl.carStop()
    mpi_ctrl.close()