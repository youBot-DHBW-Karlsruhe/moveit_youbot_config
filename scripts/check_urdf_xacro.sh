#!/bin/bash

# check parameter existance
if [ -z "$1" ] 
then
  echo "No argument supplied!"
  echo "Usage: ./check_urdf_xacro.sh file.urdf.xacro"
else

  # run commands
  echo "Running xacro.py on $1 and checking urdf syntax:"
  rosrun xacro xacro.py "$1" > tmp.urdf 
  check_urdf tmp.urdf 
  rm tmp.urdf
  echo "Finished!"

fi

