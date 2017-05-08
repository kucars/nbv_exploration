#!/usr/bin/env python

import rospy
from nbv_exploration.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
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

dir = '/home/abdullah/2017-05-06'
methods = {}

class RunStats(object):
  def __init__(self, name, path):
    self.name = name
    self.path = path
    self.final_iterations = 0
    self.final_distance = 0
    self.final_IG = 0
    self.final_coverage = 0

    self.iterations = []
    self.entropy_total = []
    self.distance = []
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

  # Read off data
  for row in c_reader:
    iterations.append( int(row[0]) )
    entropy_total.append( float(row[1]) )
    distance.append( float(row[3]) )
    utility_max.append( float(row[4]) )

  # Create final stats
  stats = RunStats(file_no_ext, file_path)
  stats.final_iterations = iterations[-1]
  stats.final_distance = distance[-1]
  stats.final_IG = entropy_total[0] - entropy_total[-1]

  stats.iterations = iterations
  stats.entropy_total = entropy_total
  stats.distance = distance
  stats.utility_max = utility_max

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
  print ("Criterion, Iterations, Distance, IG, Coverage")

  for m in methods:
    avg_IG = 0
    avg_iteration = 0
    avg_distance = 0

    for r in methods[m]:
      avg_IG += r.final_IG
      avg_distance += r.final_distance
      avg_iteration += r.final_iterations

    avg_IG /= len(methods[m])
    avg_distance /= len(methods[m])
    avg_iteration /= len(methods[m])

    print(m + ", " +
          str(avg_iteration) + ", " +
          str(avg_distance) + ", " +
          str(avg_IG)
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



  pyplot.show()

if __name__ == '__main__':
  main()

"""
f, ax_plot = pyplot.subplots(3,1)

# Define as dict so we can support values from multiple methods
iterations = {}
total_entropy = {}
max_utility = {}
med_utility = {}
distance = {}
file_prefix = time.strftime("%Y-%m-%d_%H-%M-%S_", time.localtime())
files_csv = {} #Array of open csv files

def main():
  global file_prefix
  if ( len(sys.argv) > 1):
    file_prefix = sys.argv[1] + "_" + file_prefix

  rospy.init_node('plot_iteration_info', anonymous=True)
  rospy.Subscriber("nbv_exploration/iteration_info", IterationInfo, callback)

  try:
    while (True):
        ax_plot[0].clear()
        ax_plot[1].clear()
        ax_plot[2].clear()

        for key in iterations:
          ax_plot[0].plot(iterations[key], total_entropy[key], label=key)
          ax_plot[1].plot(iterations[key], max_utility[key], label=key)
          ax_plot[2].plot(iterations[key], distance[key], label=key)

        ax_plot[0].set_ylabel('Global Entropy')
        ax_plot[1].set_ylabel('Max Utility')
        ax_plot[2].set_ylabel('Distance (m)')
        ax_plot[2].set_xlabel('Iterations')

        # Add legends
        legends = []
        legends.append( ax_plot[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )
        #legends.append( ax_plot[1].legend(loc='center left', bbox_to_anchor=(1, 0.5), shadow=True) )
        #legends.append( ax_plot[2].legend(loc='center left', bbox_to_anchor=(1, 0.5)) )

        # Set the fontsize
        for leg in legends:
          if leg is not None:
            for label in leg.get_texts():
              label.set_fontsize('small')

        pyplot.pause(1)


  except KeyboardInterrupt:
    print ("Exitting plotting node")


def callback(data):
  method = data.method

  # Check if method is already defined as key in dict
  if not method in iterations:
    # Create blank arrays
    iterations[method] = []
    total_entropy[method] = []
    max_utility[method] = []
    med_utility[method] = []
    distance[method] = []

    # Open csv file in append mode
    files_csv[method] = open(file_prefix + method + ".csv", "a")
    csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    csvwriter.writerow([
      'Iteration',
      'Entropy Total',
      'Entropy Change %',
      'Distance Travelled',
      'Utility Max',
      'Utility Median',
      'Count',
      'Utility 1', 'Utility 2', 'Utility 3', '...'
      ])

  # Iterations went back in time. Indicates start of new NBV loop. Exit program
  if (len(iterations[method]) > 1 and
      data.iteration < iterations[method][-1]):
      exit_gracefully()

  iterations[method].append(data.iteration)
  total_entropy[method].append(data.total_entropy)
  max_utility[method].append(data.max_utility)
  med_utility[method].append(data.med_utility)
  distance[method].append(data.total_distance)

  entropy_change = '';
  if (len(total_entropy[method]) > 1):
    prev = total_entropy[method][-2]
    curr = total_entropy[method][-1]
    entropy_change = (curr - prev)/((curr + prev)/2) * 100

  csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
  csvwriter.writerow([
    data.iteration,
    data.total_entropy,
    entropy_change,
    data.total_distance,
    data.max_utility,
    data.med_utility,
    len(data.utilities)
    ]
    + list(data.utilities) # Convert tuple to list
    )

def cleanup_before_exit():
  print("[Plot] Closing all csv files")
  # Close any open csv files
  for key, file in files_csv.items():
    file.close()

def exit_gracefully(signum = None, frame = None):
  cleanup_before_exit()
  os._exit(1)

if __name__ == '__main__':
  # Workaround to force plots to close when pressing CTRL-C
  original_sigint = signal.getsignal(signal.SIGINT)
  signal.signal(signal.SIGINT, exit_gracefully)

  try:
    main()
  except:
    cleanup_before_exit()
    print("Exception: " + sys.exc_info()[1])
    print('Application terminated')
"""
