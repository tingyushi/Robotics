

if __name__ == "__main__":
    import time 
    import sys ; sys.path.append("../")
    from mpi_controller_class import MegaPiController
    
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(3)

    # front left wheel forward rotate: start to rotate at 28
    # for i in range(15, 31):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vfl = i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)



    # # front left wheel backward rotate: start to rotate at -26
    # for i in range(15, 31):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vfl = -i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)


    # # front right wheel forward rotate: start to rotate at 29
    # for i in range(30, 51):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vfr = i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)


    # # front right wheel backward rotate: start to rotate at -29
    # for i in range(25, 51):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vfr = -i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)


    # # back right wheel forward rotate: start to rotate at 22
    # for i in range(15, 31):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vbr = i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)    


    # # back right wheel backward rotate: start to rotate at -17
    for i in range(25, 41):
        print(f"===== i = {i} =====")
        mpi_ctrl.setFourMotors(vbr = -i)
        time.sleep(3)
        mpi_ctrl.carStop()
        time.sleep(3)    
    

    # # back left wheel forward rotate: start to rotate at 39
    # for i in range(15, 31):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vbl = i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)    


    # # back left wheel backward rotate: start to rotate at -37
    # for i in range(15, 31):
    #     print(f"===== i = {i} =====")
    #     mpi_ctrl.setFourMotors(vbl = -i)
    #     time.sleep(3)
    #     mpi_ctrl.carStop()
    #     time.sleep(3)

    mpi_ctrl.close()