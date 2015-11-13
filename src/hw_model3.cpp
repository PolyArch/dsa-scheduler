string hw_model = R"(






* THE GOAL OF THIS VERSION IS TO REPLACE MVN WITH MEL, TO ADD ROBUST TIMING HINTS


Variable            cost,length;
binary variable     Mn(v,n), Mel(e,l), Nl(l), Mp(pv,pn);
positive variable   Sll(l,l),  Tl(l), Tn(n), Tv(v), extra(e), O(l);

Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
*Mvl.prior(v,l)=100;
*Sll.prior(l,l)=5;
Nl.prior(l)=5;

Mel.up(e,l)=1;
Nl.up(l)=1;

alias(v1,v2,v);
alias (l1,l2,l);
set Gvv(v,v);
Gvv(v1,v2)=YES$(sum(e,Gve(v1,e) and Gev(e,v2))); 
set Hll(l,l);
Hll(l1,l2)=YES$( sum(n,Hln(l1,n) and Hnl(n,l2)) or (sum(r,Hlr(l1,r) and Hrl(r,l2)) and not sum(n,Hnl(n,l1) and Hln(l2,n))) );
set Rll(l,l);
Rll(l1,l2)=YES$(sum(r,Hlr(l1,r) and Hrl(r,l2)));

set InputL(l);
InputL(l)$(sum(n$kindN('Input',n),Hnl(n,l)))=Yes;
set OutputL(l);
OutputL(l)$(sum(n$kindN('Output',n),Hln(l,n)))=Yes;

*fix v->l mappings, like no non-inputs on input links
set cl(v,l);
cl(v,l)$(not (kindV('Output',v) or (InputL(l) and not kindV('Input',v))))=YES;
loop((v1,v2)$(Gvv(v1,v2) and not kindV('Output',v2)),
  cl(v1,l)$(OutputL(l))=NO;
);
Mel.fx(e,l)$(sum(v,Gve(v,e) and not cl(v,l)))=0;


* generate compatibility matrix
set c(v,n);
loop(k,
    c(v,n)$(kindV(K,v) and kindN(K,n))=YES;
);

parameter FULAT(n);
scalar stop/0/;
loop(n,
  stop=0;
  loop((v,e)$(Gve(v,e) and c(v,n)and stop=0),
    FULAT(n) = delta(e);
    stop=1;
  );
);

* Set not-possible variables to 0
Mn.fx(v,n)$(not c(v,n))=0;
Mp.fx(pv,pn)$(not cp(pv,pn))=0;

* Set input latencies
Tl.fx(l)$(sum(n,Hnl(n,l) and kindN('Input',n)))=0;



Equations
    assignVertex(K,v)
    oneVperN(n)
    source_mapping(e,n,v)
    dest_mapping(e,n,v)
    obj;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..                      sum(v,               Mn(v,n)) =l= 1;

equation assignPort(pv), onePVperPN(pn);
assignPort(pv)..                               sum(pn$cp(pv,pn),Mp(pv,pn)) =e= 1;
onePVperPN(pn)..                               sum(pv$cp(pv,pn),Mp(pv,pn)) =l= 1;

equation vectorPorts(pv,pn,v,n);
vectorPorts(pv,pn,v,n)$(cp(pv,pn) and VI(pv,v)<>0 and PI(pn,n)<>0 and VI(pv,v)=PI(pn,n)).. Mp(pv,pn) =e= Mn(v,n);

*can only route if something routed on inputs
equation mustMapVL(l1,l2);
mustMapVL(l1,l2)$(Rll(l1,l2))..     Sll(l1,l2) =l= Nl(l1);

*can only route if something routed on outputs
equation mustMapVL2(l1,l2);
mustMapVL2(l1,l2)$(Rll(l1,l2))..     Sll(l1,l2) =l= Nl(l2);

*no routing if no mappings
equation noWeirdInRoutes(l);
noWeirdInRoutes(l1)$(sum(l2$Rll(l1,l2),1))..  sum(l2$Rll(l1,l2), Sll(l1,l2)) =g= Nl(l1);

equation noWeirdOutRoutes(l);
noWeirdOutRoutes(l2)$(sum(l1$Rll(l1,l2),1))..  sum(l1$Rll(l1,l2), Sll(l1,l2)) =g= Nl(l2);

equation    calcNumMapped(l);
calcNumMapped(l)..                  sum(e,Mel(e,l)) =l= 3*Nl(l);

*Every link (from a router) should have a corresponding routing-variable set (sll) 
*equation oneSwitchOut(l);
*oneSwitchOut(l2)$(sum(l1$Rll(l1,l2),1)>0).. sum(l1$Rll(l1,l2),Sll(l1,l2)) =e= Nl(l2);

equation forceForward(l,l,e);
forceForward(l1,l2,e)$(Rll(l1,l2)).. Mel(e,l1) + Sll(l1,l2) =l= Mel(e,l2) + 1;

equation forceBackward(l,l,e);
forceBackward(l1,l2,e)$(Rll(l1,l2)).. Mel(e,l2) + Sll(l1,l2) =l= Mel(e,l1) + 1;

equation incoming_links(e,r);
incoming_links(e,r)..   sum(l$Hlr(l,r),Mel(e,l)) =e= sum(l$Hrl(r,l), Mel(e,l));


*source_mapping(v1,n,v2)$(Gvv(v1,v2))..  Mn(v1,n) =e= sum(l$Hnl(n,l),Mel(v1,l));
*dest_mapping(v1,n,v2)$(Gvv(v1,v2))..    sum(l$Hln(l,n),Mvl(v1,l)) =g= Mn(v2,n); not working here

source_mapping(e,n,v)$(Gve(v,e))..  sum(l$Hnl(n,l),Mel(e,l)) =e= Mn(v,n);
dest_mapping(e,n,v)$(Gev(e,v))..    sum(l$Hln(l,n),Mel(e,l)) =e= Mn(v,n);

*equation timeL(l,l), timeH(l,l);
*timeL(l1,l2)$(Rll(l1,l2)).. Tl(l1) + 1 - (1-Sll(l1,l2))*50 =l= Tl(l2);
*timeH(l1,l2)$(Rll(l1,l2)).. Tl(l1) + 1 + (1-Sll(l1,l2))*50 =g= Tl(l2);
*
*equation timeInFU(n,l);
*timeInFU(n,l)$(Hln(l,n)).. Tn(n) =e= Tl(l);
*
*equation timeOutFU(n,l);
*timeOutFU(n,l)$(Hnl(n,l)).. Tl(l) =e= Tn(n) + FULAT(n);

*equation incoming_links(v,r,l);
*incoming_links(v,r,l2)$(Hrl(r,l2))..   sum(l1$Hlr(l1,r),Mvl(v,l1)) =g= Mvl(v,l2);



*not necessary, routing performed by sll
*equation incoming_links(e,r) outgoing_links(e,r)
*incoming_links(e,r)..   sum(l$Hlr(l,r),Ml(e,l)) =e= sum(l$Hrl(r,l), Ml(e,l));
*outgoing_links(e,r)..   sum(l$Hlr(l,r),Ml(e,l)) =l= 1;


*limits incomming vertices to one, but that's stupid and not what I want
*equation limit_inc_v(v,r);
*limit_inc_v(v,r)..   sum(l$Hlr(l,r),Mvl(v,l)) =l= 1;

*equation io_mapping2(e,n);
*io_mapping2(intedges,n)$(KindN('Input',n) or KindN('Output',n))..
*                        sum(l$Hln(l,n),Ml(intedges,l)) =l= 1;                        

equation latency(v,e,v);
latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     Tv(v2) =e= Tv(v1) + sum(l,Mel(e,l)) + delta(e);
  %+ extra(e);


*equation restrict_extra(e);
*restrict_extra(e)..     extra(e) =l= sum(l,Mel(e,l));

equation add(v);
add(v)..                length =g= Tv(v);


equation block_cycles(l,l);
block_cycles(l1,l2)$Rll(l1,l2).. O(l1) + CARD(L) *(Sll(l1,l2) ) - CARD(L) +1 =l= O(l2);
O.up(l)=CARD(L);


*equation out_links(l,n);
*out_links(l,n)$(Hln(l,n) and kindN('Output',n)).. cost =g= Tl(l);
*obj.. cost =e= cost;

*obj.. cost =e= sum(l,Nl(l));
obj.. cost =e= length;


Model   schedule  / all /;

solve   schedule    using mip minimizing cost;

schedule.optfile=1;
*schedule.prioropt=1;
schedule.threads=7;
*schedule.reslim=100;
schedule.holdFixed=1;

file optfile /cplex.opt/;      

put optfile;
*put 'mipemphasis 2'/;
put 'parallelmode -1'/;
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
putclose;

*display Sll.l;
*display Mvl.l;

display schedule.numEqu;
display schedule.etSolve;
display schedule.numVar;
   
)";

