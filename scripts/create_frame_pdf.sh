#!/bin/bash

echo "Creating PDF file:"
rosrun tf view_frames
echo "Removing .gv-File and opening PDF"
rm frames.gv
evince frames.pdf
echo "Finished!"
