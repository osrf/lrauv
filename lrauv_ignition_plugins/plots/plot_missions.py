#!/usr/bin/env python3

# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation

# Plot data from missions
# Usage:
#   $ python3 plot_missions.py <mission_name> <tmp>
#
# Arguments:
#
# * <mission_name>: Optional. Defaults to `testYoYoCircle`
# * <tmp>: Optional. If the string `tmp` is added, `mission/tmp/tmp.csv` is
#          used. If not present, defaults to the timestamp in
#          `missions/<mission_name>/plot_input_ref.txt`

import os
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np


def read_input_list(filename):

  timestamps_file = open(filename)
  timestamps = timestamps_file.readlines()
  timestamps = [stamp.strip() for stamp in timestamps]

  return timestamps


# Returns a list of values
# filename: Name of csv file. First row is headers
def read_csv(filename, field):

  print('Reading %s from %s' % (field, filename))

  # Ordered list, to keep indices in tact for retrieving assignment results
  vals = []

  with open(filename, 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    fieldnames = reader.fieldnames
    for row in reader:
      vals.append(row[field])

  return vals


# infile: Path to csv file
# xvals: List containing x values to plot
# yvarname: y-variable name, header field in csv colume
# ax: Matplotlib axis object
def read_and_plot_one_variable(infile, xvals, yvarname, ax, lbl, color=None):

  yvals = read_csv(infile, yvarname)

  # Eliminate the unit at the end, save just the number
  units = yvals[len(yvals)-1].split()[1]
  yvals = [float(val.split()[0]) for val in yvals]

  line, = ax.plot(xvals, yvals, color=color, alpha=0.5)
  line.set_label(lbl)

  #plt.xticks(rotation=25)
  ax.set_title(yvarname)
  ax.set_ylabel(yvarname + ' (' + units + ')')
  ax.set_xlabel('Time (s)')


def main():

  args = sys.argv[1:]

  missionName = 'testYoYoCircle'
  usingTmp = False

  if len(args) >= 1:
    missionName = args[0]

  if len(args) >= 2 and args[1] == 'tmp':
    usingTmp = True

  this_path = os.path.dirname(os.path.realpath(__file__))
  missions_path = os.path.join(this_path, 'missions')

  blue = '#4682B4'
  orange = '#FF8C00'

  # Uncomment the mission you want to plot. Plot configurations like axes,
  # labels, and legends are different for each mission.

  # VBS
  if missionName == 'testDepthVBS':
    var = [
      'depth',
      'VerticalControl.depthCmd',
      'platform_buoyancy_position',
      'VerticalControl.buoyancyAction',
    ]
    varaxs = [0, 0, 1, 1]
    # Input data
    if not usingTmp:
      timestamps = read_input_list(os.path.join(missions_path, missionName,
        'plot_input_ref.txt'))
    lbls = [
      'state', 'cmd',
      'state', 'cmd',
    ]
    colors = [orange, blue, orange, blue]
    nPlots = max(varaxs) + 1

  # Mass shifter
  elif missionName == 'testPitchMass':
    var = [
      'platform_mass_position',
      'VerticalControl.massPositionAction',
      'platform_pitch_angle',
    ]
    # Subplot to put each variable
    varaxs = [0, 0, 1]
    # Input data
    if not usingTmp:
      timestamps = read_input_list(os.path.join(missions_path, missionName,
        'plot_input_ref.txt'))
    # Legend label for axis [0]
    lbls = ['state', 'cmd', 'state']
    # Color for each variable
    colors = [orange, blue, orange]
    nPlots = max(varaxs) + 1

  # Mass shifter + VBS
  elif missionName == 'testPitchAndDepthMassVBS':
    var = [
      'depth',
      'VerticalControl.depthCmd',
      'platform_buoyancy_position',
      'VerticalControl.buoyancyAction',
      'platform_mass_position',
      'VerticalControl.massPositionAction',
      'platform_pitch_angle',
    ]
    varaxs = [0, 0, 1, 1, 2, 2, 3]
    # Input data
    if not usingTmp:
      timestamps = read_input_list(os.path.join(missions_path, missionName,
        'plot_input_ref.txt'))
    lbls = [
      'state', 'cmd',
      'state', 'cmd',
      'state', 'cmd',
      'state',
    ]
    colors = [orange, blue, orange, blue, orange, blue, orange]
    nPlots = max(varaxs) + 1

  # Yoyo
  elif missionName == 'testYoYoCircle':
    var = [
      'depth',
      'VerticalControl.elevatorAngleAction',
      'platform_pitch_angle',
    ]
    # Subplot to put each variable
    varaxs = [0, 1,  2]
    colors = None
    nPlots = max(varaxs) + 1
    # Input data
    if not usingTmp:
      timestamps = read_input_list(os.path.join(missions_path, missionName,
        'plot_input_ref.txt'))
      # Legend label for axis [0]
    lbls = ['state', 'state', 'state']

  # Sanity check for user error
  if var is None:
    print('ERROR: unsupported mission [%s]' % (missionName))
    return

  title_suffix = ''
  if usingTmp:
    title_suffix = 'Automated Test'
  elif len(timestamps) == 1:
    title_suffix = timestamps[0]
  else:
    title_suffix = timestamps[0][:8] + 'multiRuns'

  # Output directory for plot image
  out_path = missionName
  out_path = os.path.join(missions_path, out_path)
  if not os.path.exists(out_path):
    os.makedirs(out_path)


  # Arg is ratio of height to width
  w, h = plt.figaspect(nPlots * 0.73)
  fig, axs = plt.subplots(nPlots, 1, figsize=(w, h), dpi=300)

  # Space between subplots
  plt.subplots_adjust(hspace=1)

  # Plot each mission log file
  infile = os.path.join(missions_path, 'tmp', 'tmp.csv')
 

  # x-axis is time
  secs = read_csv(infile, 'EpochSeconds')
  # Convert string to numbers
  secs = np.array([float(sec) for sec in secs])
  # Offset from first timestamp, so can have some reasonable xticks
  secs = secs - secs[0]

  lbl = ''
  # Legend per file
  #if missionName == 'testYoYoCircle':
  #  lbl = lbls[t_i]

  # Plot each variable in the corresponding subplot
  for v_i in range(len(var)):
    # Legend by variable name
    #if missionName != 'testYoYoCircle':
    lbl = lbls[v_i]

    color = None
    if colors != None:
      color = colors[v_i]
    read_and_plot_one_variable(infile, secs, var[v_i], axs[varaxs[v_i]], lbl,
      color)

  # Only need legend on one subplot, all the colors are the same order
  axs[0].legend()

  # Overall figure title
  fig.suptitle(missionName + '\n' + title_suffix )
  fig.savefig(os.path.join(out_path, title_suffix + '_' + missionName + '.png'), bbox_inches='tight')

if __name__ == '__main__':
  main()
