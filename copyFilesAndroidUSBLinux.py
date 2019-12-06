print("Starting to copy files from Android -> USB -> Linux")

import glob
import os 
import shutil

UID = os.getuid()
hostname_mtp_root = glob.glob("/run/user/" + str(UID) + "/gvfs/*")[0]

file_waypoints = glob.glob(hostname_mtp_root + "/Phone/data.csv")[0]
print(file_waypoints)

shutil.copyfile(file_waypoints, "/home/cc/ee106a/fa19/class/ee106a-afy/Desktop/data.csv")
