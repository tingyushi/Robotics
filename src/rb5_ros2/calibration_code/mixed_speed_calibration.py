"""
This file include code that control the robot motors
"""


if __name__ == "__main__":
    import time 
    import sys ; sys.path.append("../")
    from mpi_controller_class import MegaPiController

    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(2)

    v_x = 0.3
    v_y = 0.3
    w_z = 0.3
    
    mpi_ctrl.carMixed(v_x, v_y, w_z)
    time.sleep(5)

    mpi_ctrl.carStop()
    mpi_ctrl.close()