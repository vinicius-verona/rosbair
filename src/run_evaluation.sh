#!/bin/bash

# Run the detection node on all bag files in a directory
# first argument is the path to bag file directory

if [ $# -eq 0 ]; then
    echo "No arguments supplied, please provide path to bag file directory, e.g. "
    echo "$0 ~/catkin_ws/Data/recordings-2"
    exit 1
fi

# sanitize input - remove trailing slash
input_dir=$(echo "$1" | sed 's:/*$::')

if [ ! -d "$input_dir" ]; then
    echo "Input bag file directory does not exist: $1"
    exit 1
fi

is_running=$(pgrep -f robot_moving_node | wc -l)
if [ "$is_running" -eq 0 ]; then
    # prefer the user to launch it themselves, rather than managing a background process
    echo "You are not running the robot_moving_node!" >&2
    echo "Please run it in a separate terminal, e.g.:" >&2
    echo "rosrun follow_me robot_moving_node" >&2
    exit 1
fi
catkin_dir=~/catkin_ws

# build but in subshell
(cd "$catkin_dir" && catkin_make) || exit 1

eval_dir=~/catkin_ws/eval

# if evaluation directory does not exist, create it
if [ ! -d "$eval_dir" ]; then
    mkdir -p "$eval_dir"
    echo "Created evaluation dir: $eval_dir"
fi

tmp_file=persons_detected.csv
if [ -f "$tmp_file" ]; then
    rm "$tmp_file"
    echo "Removed old temporary output file $tmp_file"
fi

is_running=$(pgrep -f detection_node | wc -l)
if [ "$is_running" -eq 1 ]; then
    echo "detection_node already running, kill it by:">&2
    echo "pkill detection_node">&2
    exit 1
fi

# Run the detection node, redirect output to a file "out.csv", and errors to "err.txt"
rosrun follow_me detection_node "$file_path" & # -- 2>"$out_file" & # 2>"$err_file" &
rosrun_pid=$!; 

for file_path in "$input_dir"/*.bag; do
    # get filename without path
    filename=$(basename -- "$file_path")
    echo "Processing $filename"

    rosbag info "$file_path" | grep 'duration'

    out_file="$eval_dir"/persons_"$filename".csv
    # currently, output goes to stderr to avoid buffering
    # err_file="$eval_dir"/$filename.err
    # err_file=/dev/null

    rosbag play --quiet "$file_path"

    # no explicit waiting needed
    # rosbag_pid=$!
    # wait $rosbag_pid;

    kill -2 $rosrun_pid  # send SIGINT (but not to terminate)

    # wait for file to be written
    while [ ! -f "$tmp_file" ]; do
        sleep 0.1
        # write dot without newline
        echo -n "."
    done
    echo ""

    mv -f "$tmp_file" "$out_file"

    # count lines in out.csv
    lines=$(wc -l < "$out_file")
    echo "$filename: $lines lines"
    if [ "$lines" -lt 2 ]; then
        echo "No data in $out_file, skipping plotting">&2
        continue
    fi

    # plot results
    python3 ~/catkin_ws/src/follow_me/src/evaluation.py "$out_file";
    
done

kill -9 $rosrun_pid
