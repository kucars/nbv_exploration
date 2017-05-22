#!/usr/bin/env python

import rospy
from nbv_exploration.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
import time
import signal
import sys
import csv
import os

from matplotlib import pyplot

f, ax_plot = pyplot.subplots(4,2)

# Define as dict so we can support values from multiple methods
file_prefix = time.strftime("%Y-%m-%d_%H-%M-%S_", time.localtime())
files_csv = {} #Array of open csv files

iterations = {}
distance = {}
distance_inc = {}
entropy_total = {}
density_avg = {}
time_iteration = {}
time_generation = {}
time_selection = {}
time_mapping = {}
time_termination = {}
selected_utility = {}
selected_utility_density = {}
selected_utility_entropy = {}
selected_utility_prediction = {}
selected_utility_occupied_voxels = {}

def main():
  global file_prefix
  if ( len(sys.argv) > 1):
    file_prefix = sys.argv[1] + "_" + file_prefix

  rospy.init_node('plot_iteration_info', anonymous=True)
  rospy.Subscriber("nbv_exploration/iteration_info", IterationInfo, callback)

  try:
    while (True):
        ax_plot[0][0].clear()
        ax_plot[1][0].clear()
        ax_plot[2][0].clear()
        ax_plot[3][0].clear()
        ax_plot[0][1].clear()
        ax_plot[1][1].clear()
        ax_plot[2][1].clear()
        ax_plot[3][1].clear()

        for key in iterations:
          ax_plot[0][0].plot(iterations[key], entropy_total[key], label=key)
          ax_plot[1][0].plot(iterations[key], density_avg[key], label=key)
          ax_plot[2][0].plot(iterations[key], distance_inc[key], label=key)
          ax_plot[3][0].plot(iterations[key], time_iteration[key], label=key)

          ax_plot[0][1].plot(iterations[key], selected_utility[key], label=key)
          ax_plot[1][1].plot(iterations[key], selected_utility_density[key], label=key)
          ax_plot[2][1].plot(iterations[key], selected_utility_entropy[key], label=key)
          ax_plot[3][1].plot(iterations[key], selected_utility_prediction[key], label=key)

        ax_plot[0][0].set_ylabel('Global Entropy')
        ax_plot[1][0].set_ylabel('Avg Density (pt/vox)')
        ax_plot[2][0].set_ylabel('Distance Increment (m)')
        ax_plot[3][0].set_ylabel('Iteration Time (ms)')
        ax_plot[-1][0].set_xlabel('Iterations')

        ax_plot[0][1].set_ylabel('Utility (Total)')
        ax_plot[1][1].set_ylabel('Utility (Density)')
        ax_plot[2][1].set_ylabel('Utility (Entropy)')
        ax_plot[3][1].set_ylabel('Utility (Predict)')
        ax_plot[-1][1].set_xlabel('Iterations')

        # Add legends
        legends = []
        legends.append( ax_plot[0][0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.3), shadow=True) )
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
  method = data.method_selection + " ~ " + data.method_generation

  # Check if method is already defined as key in dict
  if not method in iterations:
    # Create blank arrays
    iterations[method] = []
    distance[method] = []
    distance_inc[method] = []
    density_avg[method] = []
    entropy_total[method] = []
    time_iteration[method] = []
    time_generation[method] = []
    time_selection[method] = []
    time_mapping[method] = []
    time_termination[method] = []
    selected_utility[method] = []
    selected_utility_density[method] = []
    selected_utility_entropy[method] = []
    selected_utility_prediction[method] = []
    selected_utility_occupied_voxels[method] = []

    # Open csv file in append mode
    files_csv[method] = open(file_prefix + method + ".csv", "a")
    csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    csvwriter.writerow([
      'Iteration',
      'Entropy Total',
      'Entropy Change %',
      'Density Avg',
      'Distance Travelled',
      'Utility (Total)',
      'Utility (Density)',
      'Utility (Entropy)',
      'Utility (Prediction)',
      'Occupied Voxels',
      'Time Iteration (ms)',
      'Time Generation (ms)',
      'Time Selection (ms)',
      'Time Mapping (ms)',
      'Time Termination (ms)',
      'Count',
      'Utility 1', 'Utility 2', 'Utility 3', '...'
      ])

  # Iterations went back in time. Indicates start of new NBV loop. Exit program
  if (len(iterations[method]) > 1 and
      data.iteration < iterations[method][-1]):
      exit_gracefully()

  iterations[method].append(data.iteration)
  entropy_total[method].append(data.entropy_total)
  distance[method].append(data.distance_total)
  density_avg[method].append(data.point_density_avg)
  time_iteration[method].append(data.time_iteration)
  time_generation[method].append(data.time_generation)
  time_selection[method].append(data.time_selection)
  time_mapping[method].append(data.time_mapping)
  time_termination[method].append(data.time_termination)
  selected_utility[method].append(data.selected_utility)
  selected_utility_density[method].append(data.selected_utility_density)
  selected_utility_entropy[method].append(data.selected_utility_entropy)
  selected_utility_prediction[method].append(data.selected_utility_prediction)
  selected_utility_occupied_voxels[method].append(data.selected_utility_occupied_voxels)

  entropy_change = '';
  if (len(entropy_total[method]) > 1):
    prev = entropy_total[method][-2]
    curr = entropy_total[method][-1]
    entropy_change = (curr - prev)/((curr + prev)/2) * 100

    distance_inc[method].append(distance[method][-1] - distance[method][-2])
  else:
    distance_inc[method].append(0)

  csvwriter = csv.writer(files_csv[method], delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
  csvwriter.writerow([
    data.iteration,
    data.entropy_total,
    entropy_change,
    data.point_density_avg,
    data.distance_total,
    data.selected_utility,
    data.selected_utility_density,
    data.selected_utility_entropy,
    data.selected_utility_prediction,
    data.selected_utility_occupied_voxels,
    data.time_iteration,
    data.time_generation,
    data.time_selection,
    data.time_mapping,
    data.time_termination,
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
  except Exception, e:
    cleanup_before_exit()
    #print("Exception: " + sys.exc_info()[1])
    print("Exception: " + str(e))
    print('Application terminated')
