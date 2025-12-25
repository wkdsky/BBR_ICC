
dir=$1

awk 'BEGIN{intt=100;st=0;endd=60*1e3;cnt++;} {cnt++;if($1>st+intt){print $1,12*cnt/($1-st); cnt=0; st=$1}}' $dir

