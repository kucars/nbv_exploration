#!/usr/bin/env python

import glob
import os
import rospkg
import subprocess
import shutil

filenames = []
# Get package path
rospack = rospkg.RosPack()
batchfolder = os.path.join(rospack.get_path('nbv_exploration'), 'batch')

# Get files in batch folder
os.chdir( batchfolder )
for file in sorted( glob.glob("*.yaml") ):
  filenames.append( file )
  print(file)

for file in filenames:
  filepath = os.path.join(batchfolder,file)
  file_no_ext = os.path.splitext(file)[0]

  # Run graphing
  proc = subprocess.Popen(['rosrun nbv_exploration plot_iteration_info.py ' + file_no_ext],
              stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
  #os.system("rosrun nbv_exploration plot_iteration_info.py &")

  # Run NBV
  os.system("roslaunch nbv_exploration nbv_loop.launch batch:=true nbv_settings_file:=" + filepath)

  # Copy results into folder for later analysis
  resultfolder = os.path.join(batchfolder, file_no_ext)
  if not os.path.exists(resultfolder):
      os.makedirs(resultfolder)

  shutil.copy2('/home/abdullah/.ros/final_octree.ot',resultfolder)
  shutil.copy2('/home/abdullah/.ros/final_cloud.pcd',resultfolder)

  """
  os.chdir( resultfolder )
  for file in sorted( glob.glob(file_no_ext+"_*.csv") ):
    csvfilepath = os.path.join(resultfolder, file)
    shutil.move(csvfilepath,resultfolder)
  """
