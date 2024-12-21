"""
This file include code that control the robot motors
"""


if __name__ == "__main__":
    import time
    import sys ; sys.path.append("../")
    from mpi_controller_class import MegaPiController

    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(2)
    
    backwardSpeeds = [63, 68, 63, 68]

    mpi_ctrl.carStraightBackward(backwardSpeeds)
    time.sleep(5)

    mpi_ctrl.carStop()
    mpi_ctrl.close()