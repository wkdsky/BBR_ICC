#!/bin/bash

mkdir PlotData/PlotCelluar/
rm PlotData/PlotCelluar/Data.txt

# #PDCC
# thr=$(awk '/Avg. Throughput:/{print $3}' evaluationLog/celluar/"$env"/*.log | awk 'BEGIN{thr=0} {thr+=$1;} END{thr=thr*8/1000/1000;print thr}')
# delayavg=$(mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/PDCC.log | awk '/Average per-packet/{print $5}')
# delay95=$(mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/PDCC.log | awk '/95th percentile/{print $6}')
# echo $delayavg $thr $delay95 >> PlotData/PlotCelluar/"$env"celluarData.txt

# #Copa
# thr=$(awk '/Avg. Throughput:/{print $3}' copaOutputLog/celluar/*.log | awk 'BEGIN{thr=0} {thr+=$1;} END{thr=thr*8/1000/1000;print thr}')
# delayavg=$(mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/Copa.log | awk '/Average per-packet/{print $5}')
# delay95=$(mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/Copa.log | awk '/95th percentile/{print $6}')
# echo $delayavg $thr $delay95 >> PlotData/PlotCelluar/"$env"celluarData.txt


#bbr Cubic
list="Cubic BBR ICC Copa"
#list="PDCC"
maxthr=0
maxavg=99999
max95=99999
llist="taxi"

for j in $llist
do
	echo CC avgDelay Throughput 95tailDelay > PlotData/"$j"Data.txt
	for i in $list
	do
		cp uplinkLog/celluar/"$i""$j".log uplinkLog/celluar/"$i""$j"_bak.log
		awk '{if($2!="d")print $0}' uplinkLog/celluar/"$i""$j"_bak.log > uplinkLog/celluar/"$i""$j".log
		thr=$(./mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/"$i""$j".log | awk '/Average throughput/{print $3}')
		delayavg=$(./mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/"$i""$j".log | awk '/Average per-packet/{print $5}')
		delay95=$(./mm-throughput-graph-lhy-pro 1 55000 uplinkLog/celluar/"$i""$j".log | awk '/95th percentile/{print $6}')
		# maxthr=$(awk 'BEGIN{if('$maxthr'>'$thr'){print '$maxthr';}else {print '$thr'}}')
		# maxavg=$(awk 'BEGIN{if('$maxavg'<'$delayavg'){print '$maxavg';}else {print '$delayavg'}}')
		# max95=$(awk 'BEGIN{if('$max95'<'$delay95'){print '$max95';}else {print '$delay95'}}')
		echo $i $delayavg $thr $delay95 >> PlotData/"$j"Data.txt
	done
done

# echo $maxthr $maxavg $max95
# j=0;

# for i in $list
# do
	# let j++
	# awk '{if(NR=='$j')print $1/'$maxavg',$2/'$maxthr',$3/'$max95'}' PlotData/PlotCelluar/Data.txt > PlotData/PlotCelluar/celluarData_norm"$j".txt
# done

