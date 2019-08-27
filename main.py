from Database import Database
from Control import Control
#from Monitor import Monitor
from Path_Planning import Path_Planning
import time
import cv2

def main():
    db = Database(gps=True,lidar=False,cam=False,imu=True)      
    db.start()

    path = Path_Planning(0,db)
    Path.make_path()
    p = Path.path
    #db.path.generate_path = p

    c = Control(db=db)
    c.start()

    while True:
        if db.flag.system_stop:
            break
        else:
            try:
                time.sleep(0.1)
                pass
            except KeyboardInterrupt:
                #cv2.destroyAllWindows()
                print("Keyboard Interrupt detected!")
                db.flag.system_stop = True
                break
    c.join()
    db.join()

    return 0

if __name__ == "__main__":
    if main() == 0:
        print("\nAutonomous-Car-System terminated successfully!")
    else:
        print("\nThere is something wrong. I recommend you to kill every processes which is related to this program.")
