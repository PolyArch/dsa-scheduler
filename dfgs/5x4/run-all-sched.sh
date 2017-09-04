#subalg="M.R.T M.RT MR.T MR.RT MR'.RT MRT'.RT  MRT"

subalg="MRT' MR' MR.RT MR'.RT MRT'.RT MRT"


logfile=log.txt
sum=summary.txt

echo -n "" | tee $sum

for i in $subalg; do
  echo $i  | tee -a $sum

  echo -e "\n\n\n\n\n\n**********          $i            *********" >> $logfile
  run-sched.sh gams $i  >> $logfile

  cat sum.txt | tee -a $sum
done

