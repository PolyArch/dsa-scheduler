file outfile / "softbrain.out" /;
outfile.pc=8;
outfile.pw=4096;
put outfile;
put "[status_message_begin_scheduling]" /

* place_heur_deform, place_heur_pos, fix_stage_1, fix_stage_2, mipstart, 

Variable            cost;
integer variable   length;
binary variable     Mn(v,n), Mel(e,l), Mp(pv,pn);

positive variable   Mvl(v,l);
positive variable   O(l);
integer variable extra(e);
*maxExtra

* Not using these variables
binary variable     PT(n);
binary variable     PTen(e,n);

binary variable   Sll(l,l);
binary variable     Nl(l);

scalar MLAT /35/;

Mvl.up(v,l)=1;

*Mvl.prior(v,l)=100;
*Mn.prior(v,n)=0;
*Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
*Mel.prior(e,l)=10;
*PT.prior(n)=10;

Mel.l(e,l)=0;

integer variable Tv(v);
integer variable minTpv(pv);
integer variable maxTpv(pv);

* generate compatibility matrix
set c(v,n);
loop(k,
        c(v,n)$(kindV(K,v) and kindN(K,n))=YES;
);

* Setup aliases and useful notational shortcuts
alias(v1,v2,v);
alias(l1,l2,l);
alias(n,n1,n2);
alias(e,e1,e2);
set Gvv(v,v);
Gvv(v1,v2)=YES$(sum(e,Gve(v1,e) and Gev(e,v2))); 
set Nll(l,l);
Nll(l1,l2)=YES$(sum(n,Hln(l1,n) and Hnl(n,l2)));
set Hll(l,l);
Hll(l1,l2)=YES$( sum(n,Hln(l1,n) and Hnl(n,l2)) or (sum(r,Hlr(l1,r) and Hrl(r,l2)) and not sum(n,Hnl(n,l1) and Hln(l2,n))) );
set Rll(l,l);
Rll(l1,l2)=YES$(sum(r,Hlr(l1,r) and Hrl(r,l2)));

set InputL(l);
InputL(l)$(sum(n$kindN('Input',n),Hnl(n,l)))=Yes;
set OutputL(l);
OutputL(l)$(sum(n$kindN('Output',n),Hln(l,n)))=Yes;
set FU(n);
FU(n)$(not KindN('Input',n) and not KindN('Output',n))=Yes;


*flexiVectorPorts are always compatible with anything
cp(pv,pn)=1; 

loop((e,v)$(KindV('Output',v) and Gev(e,v)),
  put e.tl/;
  extra.fx(e)=0;
);

$batinclude mip_start.gams


*variable    hlrmel(e,r);
*Intermediate variables in case of mipstart
*hlrmel.l(e,r) = sum(l$Hlr(l,r),Mel.l(e,l));

* Set not-possible variables to 0
loop(K, Mn.fx(v,n)$(kindV(K,v) and not kindN(K,n))=0;);
Mp.fx(pv,pn)$(not cp(pv,pn))=0;
Mvl.fx(v,l)$(kindV('Output',v))=0;

*no non-inputs on input links
Mvl.fx(v,l)$(InputL(l) and not kindV('Input',v))=0;

* no non-outputs on output links
loop(v1$(not sum(v2$Gvv(v1,v2), kindV('Output',v2))),
  Mvl.fx(v1,l)$(OutputL(l))=0;
);

PTen.fx(e,n)$(not FU(n))=0;

*PT.fx(n)$(not FU(n))=0;
PT.l(n)=0;


* Set input latencies to 0
Tv.lo(v) = minT(v);
Tv.up(v) = minT(v) + 25;
Tv.up(v)$kindV('Input',v)=0;

loop((pv,v)$(VI(pv,v) <> 0 and KindV('Output',v)),
  minTpv.lo(pv)=minT(v);
  maxTpv.lo(pv)=minT(v);

  minTpv.up(pv)=minT(v) + 25;
  maxTpv.up(pv)=minT(v) + 25;
);




equations assignPort(pv), onePVperPN(pn);
assignPort(pv)..                               sum(pn$cp(pv,pn),Mp(pv,pn)) =e= 1;
onePVperPN(pn)..                               sum(pv$cp(pv,pn),Mp(pv,pn)) =l= 1;

*equation vectorPorts(pv,pn,v,n);
*vectorPorts(pv,pn,v,n)$(cp(pv,pn) and VI(pv,v)<>0 and PI(pn,n)<>0 and VI(pv,v)=PI(pn,n)).. Mp(pv,pn) =e= Mn(v,n);
equation flexiVectorPorts(pv,pn,v);
flexiVectorPorts(pv,pn,v)$(cp(pv,pn) and VI(pv,v)<>0)..  Mp(pv,pn)=l=sum(n$(PI(pn,n)<>0),Mn(v,n));



equation orderVectorPorts(pv,pn,v,n,v,n);
orderVectorPorts(pv,pn,v1,n1,v2,n2)$(cp(pv,pn) and VI(pv,v1)<>0 and PI(pn,n1)<>0 and PI(pn,n2)<>0 and VI(pv,v2)<>0 and PI(pn,n2) < PI(pn,n1) and VI(pv,v2) > VI(pv,v1) ).. 2 - Mp(pv,pn) - Mn(v1,n1) =g= Mn(v2,n2);


equations assignVertex(K,v), oneVperN(n);

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..  sum(v,Mn(v,n)) =l= 1;
*Passthrough Version -- Delete above line
*oneVperN(n)..  sum(v,Mn(v,n)) + PT(n) =l= 1;


positive variable D(e);
equation deformation(v,e,v,n,n);
deformation(v1,e,v2,n1,n2)$(Gve(v1,e) and Gev(e,v2) and c(v1,n1) and c(v2,n2) and ORD(n1) <> ORD(n2)).. 
  D(e) =g= DIST(n1,n2) * (Mn(v1,n1) + Mn(v2,n2) - 1);



equation deformation2(v,e,v,n,n);
deformation2(v1,e,v2,n1,n)$(Gve(v1,e) and Gev(e,v2) and c(v1,n1)).. 
  d(e) =g= ORD(n) * ( Mn(v1,n1) - 1 +
      sum(n2$(c(v2,n2) and ORD(n1) <> ORD(n2) and DIST(n1,n2) = ORD(n)), Mn(v2,n2)));

positive variable PXv(v),PYv(v);
equations pos_x1(v,n), pos_x2(v,n), pos_y1(v,n), pos_y2(v,n);
pos_x1(v,n)$(c(v,n)).. PXv(v) =g= PXn(n) * Mn(v,n);
pos_x2(v,n)$(c(v,n)).. PXv(v) =l= PXn(n)  + (1 - Mn(v,n)) * 20;
pos_y1(v,n)$(c(v,n)).. PYv(v) =g= PYn(n) * Mn(v,n);
pos_y2(v,n)$(c(v,n)).. PYv(v) =l= PYn(n)  + (1 - Mn(v,n)) * 20;


positive variable DX(e), DY(e);
equations pd_x1(v,e,v), pd_x2(v,e,v), pd_y1(v,e,v), pd_y2(v,e,v);
pd_x1(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..  dx(e) =g= ( PXv(v1) - PXv(v2) + 2);
pd_x2(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..  dx(e) =g= ( PXv(v2) - PXv(v1) );
pd_y1(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..  dy(e) =g= ( PYv(v1) - PYv(v2) + 2);
pd_y2(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..  dy(e) =g= ( PYv(v2) - PYv(v1) );

equations pos_diff(e);
pos_diff(e).. d(e) =e= dx(e) + dy(e);

                                
equations sum_deformation;
sum_deformation..  cost =e= sum(e,D(e));


D.lo(e)=4;
D.lo(e)$( (sum(v$kindV('Input',v),Gve(v,e))=1) ) =2;
D.lo(e)$( (sum(v$kindV('Output',v),Gev(e,v))=1) ) =2;

*equation calc_l_used(v,l);
*calc_l_used(v,l)..                 sum(e$(Gve(v,e)),Mel(e,l)) - 25*Mvl(v,l) =l= 0;

equation opposite_calc_l_used(v,l);
opposite_calc_l_used(v,l)..  sum(e$(Gve(v,e)),Mel(e,l)) =g= Mvl(v,l);

equation calc_l_used2(v,e,l);
calc_l_used2(v,e,l)$(Gve(v,e))..   Mel(e,l) =l= Mvl(v,l);

equation    oneEperL(l);
oneEperL(l)..                  sum(v,Mvl(v,l)) =l= 1;

*integer variable NL(L);
*integer variable ML;
*NL.up(l)=4;
*ML.up=4;
*equation    oneEperL(l);
*oneEperL(l)..                  sum(v,Mvl(v,l)) =l= NL(l);
*
*equation    maxL(l);
*maxL(l)..  NL(l) =l= ML;

equation    source_mapping(e,n,v), dest_mapping(e,n,v);
source_mapping(e,n,v)$(Gve(v,e))..  sum(l$Hnl(n,l),Mel(e,l)) =e= Mn(v,n);
dest_mapping(e,n,v)$(Gev(e,v))..    sum(l$Hln(l,n),Mel(e,l)) =e= Mn(v,n);

equation    source_mapping_p(v,e,n), dest_mapping_p(e,v,n);
source_mapping_p(v1,e,n)$(Gve(v1,e)).. sum(l$Hnl(n,l),Mel(e,l)) =e= 
                                                                Mn(v1,n) + PTen(e,n);
dest_mapping_p(e,v2,n)$(Gev(e,v2))..   sum(l$Hln(l,n),Mel(e,l)) =e= 
                                                                Mn(v2,n) + PTen(e,n);

equation    no_fu_router_loop(n,r,l1,l2,e);
no_fu_router_loop(n,r,l1,l2,e)$(Hnl(n,l1) and Hlr(l1,r) and Hrl(r,l2) and Hln(l2,n))..
                          Mel(e,l1) + Mel(e,l2) =l= 1;


equations incoming_links(e,r),  outgoing_links(e,r);
incoming_links(e,r)..   sum(l$Hlr(l,r),Mel(e,l)) =e= sum(l$Hrl(r,l), Mel(e,l));
outgoing_links(e,r)..   sum(l$Hlr(l,r),Mel(e,l)) =l= 1;

* must not reconverge edges!
equation limit_inc_v(v,r);
limit_inc_v(v,r)..   sum(l$Hlr(l,r),Mvl(v,l)) =l= 1;


equation latency_relaxed(v,e,v);
latency_relaxed(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     
  Tv(v2) =g= Tv(v1) + sum(l,Mel(e,l)) + delta(e) - 1;

equation latency(v,e,v);
latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     
  Tv(v2) =e= Tv(v1) + sum(l,Mel(e,l)) + delta(e) + extra(e) - 1;

equation set_max_extra(e,v);
set_max_extra(e,v)$((not KindV('Output',v)) and Gev(e,v)).. 
        extra(e) =l= max_edge_delay * (1 + sum(n,PTen(e,n)));

*Define minimum and maximum times for vector ports
Equations min_pv(pv,v),dist_pv(pv);
Equations max_pv(pv,v), add(pv);

min_pv(pv,v)$(VI(pv,v) <> 0 and KindV('Output',v)).. minTpv(pv) =l= Tv(v);
max_pv(pv,v)$(VI(pv,v) <> 0 and KindV('Output',v)).. maxTpv(pv) =g= Tv(v);

* All elements of vector port arrive simultaneously
dist_pv(pv).. minTpv(pv) + 0 =g= maxTpv(pv); 
add(pv)..     length =g= maxTpv(pv);

*cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));

Equation obj;
obj.. cost =e= length;

* Code for determining an ordering for removing cycles
*O.l(l)=0;
*
*loop(l,
*    loop((l1,l2)$Hll(l1,l2),
*        O.l(l2) = max(O.l(l1) + sum(e,Mel.l(e,l1) + Mel.l(e,l2)) -1, O.l(l2));
*    );
*);

equation block_cycles(l,l,e);
block_cycles(l1,l2,e)$Hll(l1,l2).. O(l1) + MLAT *(Mel(e,l1) + Mel(e,l2) -1) - MLAT +1 =l= O(l2);
O.up(l)=MLAT;



*---   Constraints for Relaxed Routing ---*

equation noWeirdInRoutes(l);
noWeirdInRoutes(l1)$(sum(l2$Rll(l1,l2),1)).. 
                                   sum(l2$Rll(l1,l2), Sll(l1,l2)) =g= (1 - Nl(l1));

*can only route if something routed on inputs
equation forceBackward(l,l,e);
forceBackward(l1,l2,e)$(Rll(l1,l2)).. Mel(e,l2) + Sll(l1,l2) =l= Mel(e,l1) + 1;

equation block_cycles_sll(l,l);
block_cycles_sll(l1,l2)$Rll(l1,l2).. O(l1) + MLAT *(Sll(l1,l2) ) - MLAT +1 =l= O(l2);

equation block_cycles_sll2(l,l);
block_cycles_sll2(l1,l2)$Nll(l1,l2).. O(l1) + 1 =l= O(l2);

equation    calcNumMapped2(l);
calcNumMapped2(l)..                  sum(v,Mvl(v,l)) =e= (1 - Nl(l));

equation    one_source(l);
one_source(l2)$((sum(l1$Rll(l1,l2),1))).. sum(l1$Rll(l1,l2), Sll(l1,l2)) + Nl(l2)=e=1;

*---  ----------------------------  ---*

option threads=8;


Model fus_ok / assignVertex, oneVperN, obj /;
Model ports_ok / assignVertex, oneVperN,assignPort,onePVperPN,orderVectorPorts,flexiVectorPorts, obj  /;


file optfile /gurobi.opt/;      
file optfile2 /cplex.opt/;      


if(stages('mipstart'),

put "[status_message_fus_ok]" /;
put "[status_message_ports_ok]" /;

put optfile;
put 'mipstart 1'/;
putclose;
put optfile2;
put 'mipstart 1'/;
putclose;
put outfile;


else 

solve fus_ok    using mip minimizing cost;
put "[status_message_fus_ok]" /;

solve ports_ok    using mip minimizing cost;
put "[status_message_ports_ok]" /;

);


Model map_heur / deformation, sum_deformation, assignVertex, oneVperN,assignPort,onePVperPN,orderVectorPorts,flexiVectorPorts /;

  if(stages('place_heur_deform'),  
  map_heur.holdfixed=1;
  if(stages('mipstart'),
    map_heur.optfile=1;
  );

  solve map_heur using mip minimizing cost;
  
  put "[status_message] M. (deform)" /;
  
  if(map_heur.Modelstat gt 2 and map_heur.Modelstat <> 7 and map_heur.Modelstat <> 8,
    put "[SCHEDULE FAILED]" /; abort "schedule failed";
  );

);



Model map_heur2 / pos_x1, pos_x2, pos_y1, pos_y2, pd_x1, pd_x2, pd_y1, pd_y2, pos_diff, sum_deformation, assignVertex, oneVperN,assignPort,onePVperPN,orderVectorPorts,flexiVectorPorts /;

if(stages('place_heur_pos'),
  map_heur2.holdfixed=1;
  if(stages('mipstart'),
    map_heur2.optfile=1;
  );

  solve map_heur2 using mip minimizing cost;

  if(stages('mipstart'),
    map_heur2.optfile=1;
  );


  if(map_heur2.Modelstat gt 2 and map_heur2.Modelstat <>7 and map_heur2.Modelstat <> 8,
    put "[SCHEDULE FAILED]" /; abort "schedule failed";
  );
  
  
  put "[status_message] M. (pos)" /;
);

if(stages('fixM'),
  Mp.fx(pv,pn) = round(Mp.l(pv,pn));
  Mn.fx(v,n) = round(Mn.l(v,n));

  loop(e,
    if(sum(l,Mel.l(e,l))=2,
      Mel.fx(e,l)=Mel.l(e,l);
*      Sll.fx(l1,l2)$(Mel.l(e,l2))=Sll.l(l1,l2);
    );
  );

*  PTen.fx(e,n) = round(PTen.l(e,n));
*
*  Tv.fx(v) = round(Tv.l(v));
*  loop((pv,v)$(VI(pv,v) <> 0 and KindV('Output',v)),
*    minTpv.fx(pv)=round(Tv.l(v));
*    maxTpv.fx(pv)=round(Tv.l(v));
*  );


*  Mn.fx(v,n)$(not FU(n)) = round(Mn.l(v,n));
*  Mn.fx(v,n)$(PXn(n)=1) = round(Mn.l(v,n));
*  Mn.fx(v,n)$(PYn(n)=1) = round(Mn.l(v,n));
  put "[status_message] fix mapping" /;
);





Model sched_MRp / assignVertex, oneVperN,assignPort,onePVperPN,orderVectorPorts,
                    flexiVectorPorts, incoming_links, outgoing_links, 
                    no_fu_router_loop,
                    source_mapping_p, dest_mapping_p,  
                    opposite_calc_l_used, calc_l_used2, oneEperL, 
                    limit_inc_v, latency_relaxed, max_pv, add, obj /;


Model sched_MR / assignVertex, oneVperN,assignPort,onePVperPN,orderVectorPorts,flexiVectorPorts, incoming_links, outgoing_links, source_mapping, dest_mapping, opposite_calc_l_used, calc_l_used2, oneEperL, limit_inc_v, latency_relaxed, max_pv, add, obj /;




if(stages('MR'),
  if(stages('passthrough'),
    if(stages('mipstart'),
      sched_MRp.optfile=1;
    );

    sched_MRp.holdfixed=1;
    solve sched_MRp using mip minimizing cost;
    if(sched_MRp.Modelstat gt 2 and sched_MRp.Modelstat<>7 and sched_MRp.Modelstat<>8,
      put "[SCHEDULE FAILED]" /; abort "schedule failed";
    );

  else
    if(stages('mipstart'),
      sched_MR.optfile=1;
    );

    sched_MR.holdfixed=1;
    solve sched_MR using mip minimizing cost;
    if(sched_MR.Modelstat gt 2 and sched_MR.Modelstat<>7 and sched_MR.Modelstat<>8,
      put "[SCHEDULE FAILED]" /; abort "schedule failed";
    );
  );
  
  if(stages('mipstart'),
    sched_MR.optfile=1;
  );
  
  put "[status_message] MR." /;
  
  Mp.fx(pv,pn) = round(Mp.l(pv,pn));
  Mn.fx(v,n) = round(Mn.l(v,n));
  
  put "[status_message] fix mapping" /;

);


if(stages('fixR'),
  Mvl.fx(v,l) = round(Mvl.l(v,l));
  Mel.fx(e,l) = round(Mel.l(e,l));
  put "[status_message] fix routing" /;
);

option mip=gurobi;

*fix the positions of the functional units
*Mp.fx(pv,pn) = round(Mp.l(pv,pn));
*O.fx(l) = round(O.l(l));
*Tv.fx(v) = round(Tv.l(v));

display Mn.l;




Sll.fx(l,l)=0;
Sll.l(l,l)=0;
Model schedulep /assignVertex, oneVperN, incoming_links, outgoing_links, 
  assignPort, 
  onePVperPN, flexiVectorPorts, orderVectorPorts, opposite_calc_l_used,
  calc_l_used2, oneEperL, 
  no_fu_router_loop,
  source_mapping_p, dest_mapping_p, set_max_extra, 
  limit_inc_v,
  latency, min_pv, dist_pv, max_pv, add, obj, block_cycles /;

*Model schedulep /assignVertex, oneVperN, incoming_links,
*  assignPort, onePVperPN, flexiVectorPorts, orderVectorPorts, 
*  opposite_calc_l_used, calc_l_used2, oneEperL, 
*  source_mapping_p, dest_mapping_p, set_max_extra, 
*  latency, min_pv, dist_pv, max_pv, add, obj, 
*  forceBackward, calcNumMapped2, one_source, noWeirdInRoutes,
*  block_cycles_sll, block_cycles_sll2/;


*** ------------------------------- ***



Model schedule /assignVertex, oneVperN, incoming_links, outgoing_links, 
  assignPort, 
  onePVperPN, flexiVectorPorts, orderVectorPorts, opposite_calc_l_used,
  calc_l_used2, oneEperL, 
  source_mapping, dest_mapping, set_max_extra, 
  limit_inc_v,
  latency, min_pv, dist_pv, max_pv, add, obj, block_cycles /;

if(stages('MRT'),
  if(stages('passthrough'),
    if(stages('mipstart'),
      schedulep.optfile=1;
    );
    schedulep.holdfixed=1;
    solve   schedulep    using mip minimizing cost;
    if(schedulep.Modelstat gt 2 and schedulep.Modelstat<>7 and schedulep.Modelstat<>8,
      put " [SCHEDULE FAILED]" ;
      put schedulep.Modelstat /; abort "schedule failed";
    );


  else
    if(stages('mipstart'),
      schedule.optfile=1;
    );
    schedule.holdfixed=1;
    solve   schedule    using mip minimizing cost;
    if(schedule.Modelstat gt 2 and schedule.Modelstat <> 7 and schedule.Modelstat <> 8,
      put "[SCHEDULE FAILED]" /; abort "schedule failed";
    );
  );

  put "[status_message] MRT" /;
);


put "[status_message_complete]" /

loop((e,n),
  if(sum(l$Hln(l,n),Mel.l(e,l)) and sum(l$Hnl(n,l), Mel.l(e,l)),
    PT.l(n)=1;
  );
);



*schedule.prioropt=1;
*schedule.threads=8;
*schedule.reslim=100;
*schedule.holdFixed=1;


*schedule.optfile=1;
*file optfile /cplex.opt/;      
*put optfile;
*put 'mipemphasis 2'/;
*put 'parallelmode -1'/;
*put 'probe 2'/;
*put 'heurfreq 1'/;
*put 'coeredind 3'/;
*put 'mipstart 1'/;
*put 'lpmethod 3'/;
*put 'mipsearch 1'/;
*put 'bndstrenind 1'/;
*put 'symmetry 5'/;
*put 'varsel 3'/;
*put 'depind 3'/;
*put 'divetype 3'/;
*put 'cuts 1'/;
*put 'lbheur 1'/;
*put 'mipordtype 3'/;
*put 'ppriind 2'/;
*put 'nodesel 3'/;
*put 'subalg 2'/;
*put 'mipordtype 3'/;
*putclose;

*display Mel.l;
display Mvl.l;
display Sll.l;

*put schedule.Modelstat /;


*display schedule.numEqu;
*display schedule.etSolve;
*display schedule.numVar;
