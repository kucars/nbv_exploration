#!/usr/bin/env python

import rospy
from nbv_exploration.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix, isfinite
import time
import signal
import sys
import csv
import os
import glob
import subprocess
import shutil

from matplotlib import pyplot

"""
Folder structure is assumed to be the following:
- root
-- Method1
--- Run1 (contains 1 csv)
--- Run2 (contains 1 csv)
--- Run3 (contains 1 csv)
-- Method2
--- Run1
--- Run2
...

"""

dir = '/home/abdullah/NBV_Results'
methods = {}

class RunStats(object):
  def __init__(self, name, folder, path):
    self.name = name
    self.path = path
    self.folder = folder
    self.final_iterations = 0
    self.final_distance = 0
    self.final_IG = 0
    self.final_coverage = 0
    self.final_time = 0

    self.iterations = []
    self.distance = []
    self.entropy_total = []
    self.time_iteration = []
    self.utility_max = []

def ExtractRunData(folder):
  # Read first csv file
  os.chdir( folder )
  file = glob.glob("*.csv")[0]

  file_no_ext = os.path.splitext(file)[0]
  file_path = os.path.join(folder,file)

  c_reader = csv.reader(open(file_path, 'r'), delimiter=',')
  next(c_reader, None)  # skip the headers

  # Initialize columns
  iterations = []
  entropy_total = []
  distance = []
  utility_max = []
  time_iteration = []

  # Read off data
  for row in c_reader:
    iterations.append( int(row[0]) )
    entropy_total.append( float(row[1]) )
    distance.append( float(row[3]) )
    utility_max.append( float(row[4]) )

    t = 0
    try:
      t = float(row[6])
      if not isfinite(t):
        t = 0
    except:
      # Failure happens if number is blank
      print('Failed to convert time to float in iteration ' + str(iterations[-1]) )

    time_iteration.append ( t )

  # Create final stats
  stats = RunStats(file_no_ext, folder, file_path)
  stats.final_iterations = iterations[-1]
  stats.final_distance = distance[-1]
  stats.final_IG = entropy_total[0] - entropy_total[-1]

  stats.final_time = 0
  for t in time_iteration:
    stats.final_time += t

  stats.iterations = iterations
  stats.entropy_total = entropy_total
  stats.distance = distance
  stats.utility_max = utility_max
  stats.time_iteration = time_iteration

  return stats

def getMethodData():
  # Get method names
  os.chdir( dir )
  for name in os.listdir("."):
    if os.path.isdir(name):
      # Create method entry for each folder
      methods[name] = []

  # Get different runs for each method
  for m in methods:
    folder_method = os.path.join(dir,m)

    os.chdir( folder_method )
    for name in sorted( os.listdir(".") ):
      if os.path.isdir(name):
        folder_run = os.path.join(folder_method,name)
        run_stats = ExtractRunData(folder_run)
        methods[m].append( run_stats )

        os.chdir( folder_method ) # Return to method folder

def main():
  getMethodData()

  # Get number of runs
  total_runs = 0
  for m in methods:
    runs = len(methods[m])
    if (runs > total_runs):
      total_runs = runs

  # Display final statistcs
  print ("Method, Runs, Iterations, Distance (m), Entropy Reduction, Total Time (s), Avg Time Per Iteration (ms), Coverage (res = 0.05m), Coverage (res = 0.10m), Coverage (res = 0.50m)")

  for m in sorted(methods):
    avg_IG = 0
    avg_iteration = 0
    avg_distance = 0
    avg_time = 0
    avg_coverage_0_5 = 0
    avg_coverage_0_1 = 0
    avg_coverage_0_05 = 0

    for r in methods[m]:
      avg_IG += r.final_IG
      avg_distance += r.final_distance
      avg_iteration += r.final_iterations
      avg_time += r.final_time/1000

      # Get coverage for profile
      profile_path = os.path.join(r.folder,"final_cloud.pcd")
      cmd = 'rosrun nbv_exploration evaluate_coverage -b -i "' + profile_path + '" 0.5 0.1 0.05'

      # Get coverage
      process = subprocess.Popen([cmd], shell=True, stdout=subprocess.PIPE)
      coverage_results = process.communicate()[0]
      vec = [float(s.strip()) for s in coverage_results.splitlines()]

      avg_coverage_0_5 += vec[0]
      avg_coverage_0_1 += vec[1]
      avg_coverage_0_05 += vec[2]

    avg_IG /= len(methods[m])
    avg_distance /= len(methods[m])
    avg_iteration /= len(methods[m])
    avg_time /= len(methods[m])
    avg_time_per_iteration = 1000*avg_time/avg_iteration

    avg_coverage_0_5 /= len(methods[m])
    avg_coverage_0_1 /= len(methods[m])
    avg_coverage_0_05 /= len(methods[m])

    print(m + ", "
          + str(len(methods[m])) + ", "
          + str( round(avg_iteration,2) ) + ", "
          + str( round(avg_distance,2) ) + ", "
          + str( round(avg_IG,2) ) + ", "
          + str( round(avg_time,2) ) + ", "
          + str( round(avg_time_per_iteration,2) ) + ", "
          + str( round(avg_coverage_0_05,2) )  + "%, "
          + str( round(avg_coverage_0_1,2) )  + "%, "
          + str( round(avg_coverage_0_5,2) ) + "%"
          )

  # Display entropy in each run
  f, ax_plot = pyplot.subplots(total_runs,1,sharey=True)

  for r in range(total_runs):
    for key, method in methods.items():
      if (r >= len(method)):
        continue
      ax_plot[r].plot(method[r].iterations, method[r].entropy_total, label=key)
      ax_plot[r].set_ylabel('Total Entropy')
      ax_plot[r].legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)


  # Display distance in each run
  f, ax_plot = pyplot.subplots(total_runs,1)

  for r in range(total_runs):
    for key, method in methods.items():
      if (r >= len(method)):
        continue
      ax_plot[r].plot(method[r].iterations, method[r].distance, label=key)
      ax_plot[r].set_ylabel('Distance (m)')
      ax_plot[r].legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)


  # Display utility in each run
  f, ax_plot = pyplot.subplots(total_runs,1)
  f.suptitle("Utility")

  for r in range(total_runs):
    for key, method in methods.items():
      if (r >= len(method)):
        continue
      ax_plot[r].plot(method[r].iterations, method[r].utility_max, label=key)
      ax_plot[r].set_ylabel('Utility Max')
      ax_plot[r].legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)

  # Display time per iteration in each run
  f, ax_plot = pyplot.subplots(total_runs,1)
  f.suptitle("Time Per Iteration")

  for r in range(total_runs):
    for key, method in methods.items():
      if (r >= len(method)):
        continue
      ax_plot[r].plot(method[r].iterations, method[r].time_iteration, label=key)
      ax_plot[r].set_ylabel('Time per Iteration (ms)')
      ax_plot[r].legend(loc='lower left', bbox_to_anchor=(1.0, 0.0), shadow=True)



  pyplot.show()

if __name__ == '__main__':
  main()
