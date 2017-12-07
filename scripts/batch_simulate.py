#!/usr/bin/env python

import glob
import os
import rospkg
import subprocess
import shutil

def killProcessByName(scriptName):
  process = subprocess.Popen(["ps", "-eo","pid,command"], stdout=subprocess.PIPE)
  output = process.communicate()[0]
  splitted = output.rsplit('\n')
  for line in splitted:
    if scriptName in line:
      #print line
      l = line.lstrip()
      pid = l.split(' ')[0]
      #print pid
      os.system('kill -9 ' + pid)
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

  resultfolder = os.path.join(batchfolder, file_no_ext)
  if not os.path.exists(resultfolder):
    os.makedirs(resultfolder)

  # Run graphing
  proc = subprocess.Popen(['rosrun nbv_exploration plot_iteration_info.py ' + file_no_ext],
              stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

  # Run NBV
  os.system("roslaunch nbv_exploration nbv_loop.launch debug:=false batch:=true nbv_settings_file:=" + filepath)

  # Copy results into folder for later analysis
  shutil.copy2('/tmp/final_octree.ot',resultfolder)
  shutil.copy2('/tmp/final_cloud.pcd',resultfolder)

  killProcessByName("plot_iteration_info.py")
  os.system("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient")
