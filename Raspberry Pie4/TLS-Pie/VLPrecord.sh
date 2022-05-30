#!/usr/bin/env bash

# Parent directory
# DO NOT change this
DIR=$(dirname "$(readlink -f "$0")")

# Directory to dump collected data in
#DUMPDIR="/home/lipi/velodyne"

# Whether the system is shutdown when the button is pressed.
# Either 1 or 0 (true and false).
BTNPOWEROFF=0

# Directory to put temporary files
TMPDIR="/tmp/tlspie"

# Make sure directories exist
#mkdir -p $DUMPDIR
mkdir -p $TMPDIR

# Waits for a button press then starts the recording
startbutton () {
    $DIR/VLPbuttons.py
    }

startbutton

# Cleanly terminates the program
DOPOWEROFF=0
exitscript () {
    echo "$(date): SIGINT or SIGTERM detected"
    trap - SIGINT SIGTERM
    if ! [ -z ${CHILD+x} ]
    then
	echo "$(date): Terminating data logging..."
	kill $CHILD
#	wait $CHILD
    fi
       
    sleep 0.1
    if [ $DOPOWEROFF -eq 1 ]
    then
	    echo "$(date): Powering off computer..."
	    poweroff
	    
    fi
    echo "Finished with Capture. Close terminal and restart another"
    exit 1
}
trap exitscript SIGINT SIGTERM

# Waits for a button press then terminates the program
waitbutton () {
    $DIR/VLPwaitbutton.py
    if [ $BTNPOWEROFF -eq 1 ]
    then
	DOPOWEROFF=1
    fi
    exitscript
     
}

# Capture packets and write to $DUMP
echo "$(date): Recording packets from LIDAR and writing to velodyne folder"
tcpdump -w /home/lipi/velodyne/TLS_`date +%y_%m_%d_%H_%M_%S`.pcap -i eth0 &
CHILD=$!
# Wait for data logging to stop, or for a button press
waitbutton &
wait $CHILD


