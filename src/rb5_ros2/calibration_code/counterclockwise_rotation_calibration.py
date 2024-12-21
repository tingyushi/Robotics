"""
This file include code that control the robot motors
"""


if __name__ == "__main__":
    import time
    import sys ; sys.path.append("../")
    from mpi_controller_class import MegaPiController

    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(2)
    
    counterclockwiseRotationSpeeds = [80, 72, 80, 72]

    mpi_ctrl.carRotateCounterClockwise(counterclockwiseRotationSpeeds)
    time.sleep(3.1)

    mpi_ctrl.carStop()
    mpi_ctrl.close()