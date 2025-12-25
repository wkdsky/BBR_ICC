#!/bin/bash


cd ../

#IPP="192.168.100.5"
IPP=$MAHIMAHI_BASE

flow_num=$1
dur=$2
Bd=$3
Rc=$4

for ((index=1;index<=$flow_num;index++))
do
	{
		echo ICC flow$index is established with logs at $5"$index".log && 
		./sender serverip=$IPP offduration=0 onduration="$dur"000\
		cctype=icc lamda_conf=do_ss:compete:auto_theta:auto:1\
		Bd_conf="$Bd" Rc_conf="$Rc" traffic_params=deterministic,num_cycles=1 > $5"$index".log 2>&1
	}&
	
done
wait

#constant_theta
#cctype=markovian
