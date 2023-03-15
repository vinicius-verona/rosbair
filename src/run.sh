#!/bin/bash

# Run the detection node on all bag files in a directory
# first argument is the path to bag file directory

if [ $# -eq 0 ]; then
    echo "No arguments supplied, please provide path to bag file directory, e.g. "
    echo "./run.sh ~/catkin_ws/Data/recordings-2"
    exit 1
fi

if [ ! -d "$1" ]; then
    echo "Input bag file directory does not exist: $1"
    exit 1
fi


input_dir=$1

cd ~/catkin_ws && catkin_make -q && cd - || exit 1

eval_dir=~/catkin_ws/eval

# if evaluation directory does not exist, create it
if [ ! -d "$eval_dir" ]; then
    mkdir -p "$eval_dir"
    echo "Created evaluation dir: $eval_dir"
fi

echo "Verify that 'rosrun follow_me robot_moving_node' is running"

for file_path in $input_dir/*.bag; do
    # get filename without path
    filename=$(basename -- "$file_path")

    # err_file="$eval_dir"/$filename.err
    err_file=/dev/null

    out_file="$eval_dir"/persons_"$filename".csv

    # start rosbag play of the recording in the background, kill rosrun when rosbag is done
    rosbag play --quiet "$file_path" &  # --immediate 
    rosbag_pid=$!

    # Run the detection node, redirect output to a file "out.csv", and errors to "err.txt"
    rosrun follow_me detection_node "$file_path">"$out_file" 2>"$err_file" &
    rosrun_pid=$!; wait $rosbag_pid; kill $rosrun_pid

    # count lines in out.csv
    lines=$(wc -l < "$out_file")
    echo "$filename: $lines lines"

    # plot results
    python3 ~/catkin_ws/src/follow_me/src/evaluation.py "$file_path"  # todo fix UnicodeDecodeError
done
