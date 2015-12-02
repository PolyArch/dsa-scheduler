string hw_model = R"(







Variable            cost;
binary variable     Mn(v,n), Sll(l,l), NOl(l), Mp(pv,pn);
integer variable    Tl(l), Tn(n);
positive variable   Mvl(v,l), Tv(v), extra(e), En(n);

Tl.up(l)=ORD(l);
Tn.up(n)=60;

Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
*Mvl.prior(v,l)=100;
Sll.prior(l,l)=5;

Mvl.up(v,l)=1;
NOl.up(l)=1;

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
Mvl.fx(v,l)$(not cl(v,l))=0;


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

*Mvl.fx(v,l)$(kindV('Output',v))=0;
*
**no non-inputs on input links
*Mvl.fx(v,l)$(InputL(l) and not kindV('Input',v))=0;
*
** no non-outputs on output links
*loop((v1,v2)$(Gvv(v1,v2) and not kindV('Output',v2)),
*  Mvl.fx(v1,l)$(OutputL(l))=0;
*);
*
* Set input latencies
Tl.fx(l)$(sum(n,Hnl(n,l) and kindN('Input',n)))=0;



Equations
    assignVertex(K,v)
    oneVperN(n)
    source_mapping(v1,n,v2)
    dest_mapping(v1,n,v2)
    obj;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..                      sum(v,               Mn(v,n)) =l= 1;

equation assignPort(pv), onePVperPN(pn);
assignPort(pv)..                               sum(pn$cp(pv,pn),Mp(pv,pn)) =e= 1;
onePVperPN(pn)..                               sum(pv$cp(pv,pn),Mp(pv,pn)) =l= 1;

equation vectorPorts(pv,pn,v,n);
vectorPorts(pv,pn,v,n)$(cp(pv,pn) and VI(pv,v)<>0 and PI(pn,n)<>0 and VI(pv,v)=PI(pn,n)).. Mp(pv,pn) =e= Mn(v,n);

equation output_some_link(l2);
output_some_link(l2)$(sum(l1$Rll(l1,l2),1)).. sum(l1$Rll(l1,l2),Sll(l1,l2))+NOl(l2)=e=1;

*can only route if something routed on inputs
*equation mustMapVL(l1,l2);
*mustMapVL(l1,l2)$(Rll(l1,l2))..     Sll(l1,l2) =l= Nl(l1);

*can only route if something routed on outputs
*equation mustMapVL2(l1,l2);
*mustMapVL2(l1,l2)$(Rll(l1,l2))..     Sll(l1,l2) =l= Nl(l2);

*no routing if no mappings
*equation noWeirdInRoutes(l);
*noWeirdInRoutes(l1)$(sum(l2$Rll(l1,l2),1))..  sum(l2$Rll(l1,l2), Sll(l1,l2)) =g= Nl(l1);
*
*equation noWeirdOutRoutes(l);
*noWeirdOutRoutes(l2)$(sum(l1$Rll(l1,l2),1))..  sum(l1$Rll(l1,l2), Sll(l1,l2)) =g= Nl(l2);

equation    calcNumMapped(l);
calcNumMapped(l)..                  sum(v,Mvl(v,l)) =e= (1-NOl(l));

*Every link (from a router) should have a corresponding routing-variable set (sll) 
*equation oneSwitchOut(l);
*oneSwitchOut(l2)$(sum(l1$Rll(l1,l2),1)>0).. sum(l1$Rll(l1,l2),Sll(l1,l2)) =e= Nl(l2);

equation forceForward(l,l,v);
forceForward(l1,l2,v)$(Rll(l1,l2)).. Mvl(v,l1) + Sll(l1,l2) =l= Mvl(v,l2) + 1;

equation forceBackward(l,l,v);
forceBackward(l1,l2,v)$(Rll(l1,l2)).. Mvl(v,l2) + Sll(l1,l2) =l= Mvl(v,l1) + 1;

source_mapping(v1,n,v2)$(Gvv(v1,v2))..  Mn(v1,n) =e= sum(l$Hnl(n,l),Mvl(v1,l));
dest_mapping(v1,n,v2)$(Gvv(v1,v2))..    sum(l$Hln(l,n),Mvl(v1,l)) =g= Mn(v2,n);


equation timeL(l,l), timeH(l,l);
timeL(l1,l2)$(Rll(l1,l2)).. Tl(l1) + 1 - (1-Sll(l1,l2))*64 =l= Tl(l2);
timeH(l1,l2)$(Rll(l1,l2)).. Tl(l1) + 1 + (1-Sll(l1,l2))*64 =g= Tl(l2);

equation timeInFU(n,l);
timeInFU(n,l)$(Hln(l,n)).. Tn(n) =e= Tl(l);

En.up(n)=24;
En.fx(n)$(not kindN('DelayFU',n))=0;

equation timeOutFU(n,l);
timeOutFU(n,l)$(Hnl(n,l)).. Tl(l) =e= Tn(n) + FULAT(n) + En(n);

equation incoming_links(v,r,l);
incoming_links(v,r,l2)$(Hrl(r,l2))..   sum(l1$Hlr(l1,r),Mvl(v,l1)) =g= Mvl(v,l2);

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

*equation latency(v,e,v);
*latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     Tv(v2) =e= Tv(v1) + sum(l,Ml(e,l)) + delta(e) + extra(e);

*add(v)..                length =g= Tv(v);
*cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));



obj.. cost =e= sum(l,(1-NOl(l)));

*equation out_links(l,n);
*out_links(l,n)$(Hln(l,n) and kindN('Output',n)).. cost =g= Tl(l);
*obj.. cost =e= cost;


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
put 'probe 2'/;
put 'heurfreq 1'/;
*put 'coeredind 3'/;
put 'mipstart 1'/;
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

