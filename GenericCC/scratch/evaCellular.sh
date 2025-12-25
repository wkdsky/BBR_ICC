flowNum=$1

if [ -z "$flowNum" ]; then
        flowNum=10
fi

echo "Set concurrent flow number as $flowNum."

dirs=(
    uplinkLog/
    uplinkLog/celluar/
    downlinkLog/
    downlinkLog/celluar/
)

for dir in "${dirs[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Directory $dir created."
    fi
done



if [ -n $(ps -ef | awk '/receiver/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/receiver/{print $2}') >/dev/null 2>&1
fi
if [ -n $(ps -ef | awk '/iperf/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/iperf/{print $2}') >/dev/null 2>&1
fi

rm testlog
../receiver >> testlog 2>&1 &
sleep 5s\
&&
echo Copa
bash celluarCopa.sh $flowNum >> testlog 2>&1\
&&
echo End-Copa
sleep 5s\
&&
echo ICC
bash celluarICC.sh $flowNum >> testlog 2>&1\
&&
if [ -n $(ps -ef | awk '/receiver/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/receiver/{print $2}') >/dev/null 2>&1
fi
echo End-ICC

iperf -Z bbr -s -p 8888 >> testlog 2>&1 &
sleep 5s\
&&
echo BBR
bash celluarBBR.sh $flowNum >> testlog 2>&1 \
&&
if [ -n $(ps -ef | awk '/iperf/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/iperf/{print $2}') >/dev/null 2>&1
fi  
echo End-BBR

iperf -Z cubic -s -p 8888 >> testlog 2>&1 &
sleep 5s\
&&
echo Cubic
bash celluarCubic.sh $flowNum >> testlog 2>&1 \
&&
if [ -n $(ps -ef | awk '/iperf/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/iperf/{print $2}') >/dev/null 2>&1
fi  
echo End-Cubic

bash catCellular.sh >> testlog 2>&1

cd PlotData/
bash convCatCellular.sh
bash catCapacityTrace.sh ../celluar_trace/trace_taxi > cap_cellular_taxi.txt
bash plotDetail.sh


if [ -n $(ps -ef | awk '/receiver/{if($8!="awk")cnt++} END{print cnt}') ]; then
    kill -9 $(ps -ef | awk '/receiver/{print $2}') >/dev/null 2>&1
fi
echo END the Test