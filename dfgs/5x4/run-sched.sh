mkdir -p output

for i in *.dfg; do
  echo "************ $i *************"; 	
  $SS_TOOLS/bin/sb_sched $SS_TOOLS/configs/softbrain_5x4.sbmodel $i --verbose;
  #echo   $SS_TOOLS/bin/sb_sched $SS_TOOLS/configs/softbrain_5x4.sbmodel $i --verbose;
done
