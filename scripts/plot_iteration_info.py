#!/usr/bin/env python

import rospy
from nbv_exploration.msg import IterationInfo
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
import time
import signal
import sys

from matplotlib import pyplot

f, ax_plot = pyplot.subplots(3,1)

# Define as dict so we can support values from multiple methods
iterations = {}
total_entropy = {}
max_utility = {}
med_utility = {}


def main():
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
          ax_plot[2].plot(iterations[key], med_utility[key], label=key)

        ax_plot[0].set_ylabel('Global Entropy')
        ax_plot[1].set_ylabel('Max Utility')
        ax_plot[2].set_ylabel('Median Utility')
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

  iterations[method].append(data.iteration)
  total_entropy[method].append(data.total_entropy)
  max_utility[method].append(data.max_utility)
  med_utility[method].append(data.med_utility)

def exit_gracefully(signum, frame):
  sys.exit(1)

if __name__ == '__main__':
  # Workaround to force plots to close when pressing CTRL-C
  original_sigint = signal.getsignal(signal.SIGINT)
  signal.signal(signal.SIGINT, exit_gracefully)

  main()
