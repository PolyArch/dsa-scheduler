Variable            cost,length;
binary variable     Mn(v,n), Sll(l,l);
positive variable   Mvl(v,l), Nl(l), Tl(l);

Mvl.up(v,l)=1;
Nl.up(l)=1;
Tl.up(l)=1;


alias(v1,v2,v);
alias (l1,l2,l);
set Gvv(v,v);
Gvv(v1,v2)=YES$(sum(e,Gve(v1,e) and Gev(e,v2))); 
set Hll(l,l);
Hll(l1,l2)=YES$( sum(n,Hln(l1,n) and Hnl(n,l2)) or (sum(r,Hlr(l1,r) and Hrl(r,l2)) and not sum(n,Hnl(n,l1) and Hln(l2,n))) );
set Rll(l,l);
Rll(l1,l2)=YES$(sum(r,Hlr(l1,r) and Hrl(r,l2)));


Equations
    assignVertex(K,v)
    oneVperN(n)
    source_mapping(v1,n,v2)
    dest_mapping(v1,n,v2)
    obj;

Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
*Mvl.prior(v,l)=100;
*Ml.prior(e,l)=10;

* Set not-possible variables to 0
loop(K,
Mn.fx(v,n)$(kindV(K,v) and not kindN(K,n))=0;);

* generate compatibility matrix
set c(v,n);
loop(k,
    c(v,n)$(kindV(K,v) and kindN(K,n))=YES;
);


* Set input latencies
*Tv.fx(v)$kindV('Input',v)=0;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..                      sum(v,Mn(v,n)) =l= 1;

equation mustMapVL(l1,l2);
mustMapVL(l1,l2)$(Rll(l1,l2))..     Sll(l1,l2) =l= Nl(l1);

equation    calcNumMapped(l);
calcNumMapped(l)..                  sum(v,Mvl(v,l)) =e= Nl(l);

*Every link (from a router) should have a corresponding routeint-variable set (sll) 
equation oneSwitchOut(l);
oneSwitchOut(l2)$(sum(l1$Rll(l1,l2),1)>0).. sum(l1$Rll(l1,l2),Sll(l1,l2)) =e= Nl(l2);

equation forceForward(l,l,v);
forceForward(l1,l2,v)$(Rll(l1,l2)).. Mvl(v,l1) + Sll(l1,l2) =l= Mvl(v,l2) + 1;

equation forceBackward(l,l,v);
forceBackward(l1,l2,v)$(Rll(l1,l2)).. Mvl(v,l2) + Sll(l1,l2) =l= Mvl(v,l1) + 1;


source_mapping(v1,n,v2)$(Gvv(v1,v2))..  Mn(v1,n) =e= sum(l$Hnl(n,l),Mvl(v1,l));
dest_mapping(v1,n,v2)$(Gvv(v1,v2))..    sum(l$Hln(l,n),Mvl(v1,l)) =e= Mn(v2,n);

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


*positive varaible Tn(n), Tr(r), Tl(l);

*latency3(e,n,l)$(Hnl(n,l)).. Tl(l) =e= Tn(n) + 1;
*latency4(r,l)$(Hl(n,l)).. Tl(l) =e= Tn(n) + 1;

*add(v)..                length =g= Tv(v);
*cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));



obj.. cost =e= sum(l,Nl(l));


Model   schedule  / all /;

solve   schedule    using mip minimizing cost;

schedule.optfile=1;
schedule.prioropt=1;
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

display Sll.l;
display Mvl.l;

display schedule.numEqu;
display schedule.etSolve;
display schedule.numVar;
