mkdir -p output

lat=""
lat_eq=""

lat_eq="0"
time_eq="0"


for i in *.dfg; do
  echo "************ $i *************"; 	
  $SS_TOOLS/bin/sb_sched $SS_TOOLS/configs/softbrain_5x4.sbmodel $i --verbose | tee out.txt;
  lat="$lat `grep "latency:" out.txt | cut -d" " -f 2`"
  time="$time `grep "sched_time:" out.txt | cut -d" " -f 2`"
  lat_eq="$lat_eq+`grep "latency:" out.txt | cut -d" " -f 2`"
  time_eq="$time_eq+`grep "sched_time:" out.txt | cut -d" " -f 2`"

done

echo -n $lat " | "
echo $lat_eq | bc
echo -n $time " | "
echo $time_eq | bc

