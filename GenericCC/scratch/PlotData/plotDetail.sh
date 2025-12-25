#!/bin/bash

gnuplot <<EOF
# set terminal postscript eps color solid linewidth 3 "times_new_Roman" 56

set key width 2 samplen 2 
set key right top horizontal
# set size 1.6,1.5

set terminal pngcairo enhanced font 'Arial,20' size 800,600

# set output "Cellular".".eps"
set output "Cellular".".png"
set xlabel 'Time(s)'
set ylabel 'Sending rate(Mbps)'
set xrange [0:60]
# set logscale y
# set xtics 20

cclist="Cubic BBR Copa ICC"
titlelist="Cubic BBR Copa ICC"
lc1="orange purple blue red"

set style fill solid 0.5 border lt -1

plot \
"cap_cellular_taxi.txt" using (\$1/1000):(\$2) with filledcurves x1 lc rgb "grey" title "Ideal",\
for [i=1:words(cclist)] word(cclist,i)."taxi.log" using (\$1/1000):(\$2) w lp axis x1y1 lw 3 lc rgb word(lc1,i) pt 0 dt 5-i ps 4 title word(titlelist,i),\

	


EOF

