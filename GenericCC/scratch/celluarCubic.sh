#!/bin/bash
#list="taxi bus home timessquare"

flowNum=$1

if [ -z "$flowNum" ]; then
        flowNum=10
fi

list="taxi"
for scene in $list
do
	mm-delay 10 mm-link celluar_trace/trace_"$scene" 48mbps_data.trace --uplink-log="uplinkLog/celluar/Cubic"$scene".log" --downlink-log="downlinkLog/celluar/Cubic"$scene".log" --uplink-queue=droptail --uplink-queue-args="packets=100" <<EOF
	iperf -Z cubic -c 100.64.0.1 -p 8888 -i 1 -P $flowNum -t 60 
EOF

done
