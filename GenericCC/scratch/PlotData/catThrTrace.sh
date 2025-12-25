
dir=$1

awk 'BEGIN{intt=500;st=0;endd=60*1e3;cnt++;} {if($2=="+")cnt+=$3;if($1>st+intt){print $1,8*cnt/($1-st)/1000; cnt=0; st=$1}}' $dir

