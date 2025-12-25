
list="Cubic BBR ICC Copa"

for i in $list
do
	bash catThrTrace.sh ../uplinkLog/celluar/"$i"taxi.log > "$i"taxi.log 	
done

