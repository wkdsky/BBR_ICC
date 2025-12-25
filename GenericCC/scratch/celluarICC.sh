#!/bin/bash
list="taxi"
#list="timessquare"

flowNum=$1

if [ -z "$flowNum" ]; then
        flowNum=10
fi

for scene in $list
do
	# echo $scene
	mm-delay 10 mm-link celluar_trace/trace_"$scene" 48mbps_data.trace --uplink-log="uplinkLog/celluar/ICC"$scene".log" --downlink-log="downlinkLog/celluar/ICC"$scene".log" --uplink-queue=droptail --uplink-queue-args="packets=100" <<EOF
	./send_parallel_ICC.sh $flowNum 60 10 30 "scratch/uplinkLog/ICC_"$scene"_.log" &
	sleep 61s \
	&& 
	a=$(ps -ef | awk -F " " '/ICC/ {print $2}') \
	&&
	kill -9 $a >/dev/null 2>&1
EOF

done
