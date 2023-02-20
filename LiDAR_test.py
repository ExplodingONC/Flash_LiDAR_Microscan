# import system modules
import sys
import time
import datetime
# import custom modules
import LidarControl


# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

# Lidar setup
width = int(104)  # not including header pixel
height = int(80)
Ndata = int(2)
lidar = LidarControl.LidarControl(res=[width, height], Ndata=Ndata)
try:
    lidar.connect_GPIO()
    lidar.connect_sensor()
    lidar.connect_MCU()
except Exception as err:
    print("Error:", err)
    sys.exit()
else:
    print(" - LiDAR hardware connected.")

# Lidar init
try:
    lidar.reset_sensor()
    lidar.load_MCU()
    lidar.setup_sensor()
except Exception as err:
    print("Error:", err)
    sys.exit()
else:
    print(" - LiDAR ready.")


# main program
print()
try:

    print("Free Running...")
    while 1:
        # just let it free running
        pass
    
except Exception as err:
    print("Error:", err)

finally:
    # GC
    print("\n - GPIO clean up.")
    sys.exit(0)
