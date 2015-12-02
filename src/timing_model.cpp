string timing_model = R"(

Variable            cost,length;
binary variable     Mn(v,n), Mel(e,l), Mp(pv,pn);
positive variable   Mvl(v,l);
*Mvl.prior(v,l)=100;
Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
Mel.prior(e,l)=10;

integer variable   Tv(v);

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

positive variable   O(l),extra(e),maxExtra,Ev(v);

$batinclude mip_start.gams
alias (v1,v2,v);

* Set not-possible variables to 0
loop(K,
Mn.fx(v,n)$(kindV(K,v) and not kindN(K,n))=0;);
Mp.fx(pv,pn)$(not cp(pv,pn))=0;
Mvl.fx(v,l)$(kindV('Output',v))=0;

*no non-inputs on input links
Mvl.fx(v,l)$(InputL(l) and not kindV('Input',v))=0;

* no non-outputs on output links
loop((v1,v2)$(Gvv(v1,v2) and not kindV('Output',v2)),
  Mvl.fx(v1,l)$(OutputL(l))=0;
);


* Set input latencies
Tv.fx(v)$kindV('Input',v)=0;


Equations
    assignVertex(K,v)
    oneVperN(n)
    oneEperL(l)
    add(v)
    source_mapping(e,n,v)
    dest_mapping(e,n,v)
    incoming_links(e,r)
    outgoing_links(e,r)
    obj;

equation assignPort(pv), onePVperPN(pn);
assignPort(pv)..                               sum(pn$cp(pv,pn),Mp(pv,pn)) =e= 1;
onePVperPN(pn)..                               sum(pv$cp(pv,pn),Mp(pv,pn)) =l= 1;

equation vectorPorts(pv,pn,v,n);
vectorPorts(pv,pn,v,n)$(cp(pv,pn) and VI(pv,v)<>0 and PI(pn,n)<>0 and VI(pv,v)=PI(pn,n)).. Mp(pv,pn) =e= Mn(v,n);

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..                      sum(v,Mn(v,n)) =l= 1;

*equation calc_l_used(v,l);
*calc_l_used(v,l)..                 sum(e$(Gve(v,e)),Mel(e,l)) - 25*Mvl(v,l) =l= 0;

equation opposite_calc_l_used(v,l);
opposite_calc_l_used(v,l)..  sum(e$(Gve(v,e)),Mel(e,l)) =g= Mvl(v,l);

equation calc_l_used2(v,e,l);
calc_l_used2(v,e,l)$(Gve(v,e))..   Mel(e,l) =l= Mvl(v,l);

oneEperL(l)..                  sum(v,Mvl(v,l)) =l= 1;

source_mapping(e,n,v)$(Gve(v,e))..  sum(l$Hnl(n,l),Mel(e,l)) =e= Mn(v,n);
dest_mapping(e,n,v)$(Gev(e,v))..    sum(l$Hln(l,n),Mel(e,l)) =e= Mn(v,n);

incoming_links(e,r)..   sum(l$Hlr(l,r),Mel(e,l)) =e= sum(l$Hrl(r,l), Mel(e,l));
outgoing_links(e,r)..   sum(l$Hlr(l,r),Mel(e,l)) =l= 1;

* must not reconverge edges!


*experimental
equation limit_inc_v(v,r);
limit_inc_v(v,r)..   sum(l$Hlr(l,r),Mvl(v,l)) =l= 1;

*equation io_mapping2(e,n);
*io_mapping2(intedges,n)$(KindN('Input',n) or KindN('Output',n))..
*                        sum(l$Hln(l,n),Mel(intedges,l)) =l= 1;                        

equation latency(v,e,v);
latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     Tv(v2) =e= Tv(v1) + sum(l,Mel(e,l)) + delta(e) + Ev(v1) +extra(e);

Ev.up(v)=16;
Ev.fx(v)$(not kindV('DelayFU',v))=0;
*extra.fx(e)=0;



equation restrict_extra(e);
restrict_extra(e)..     extra(e) =l= sum(v$Gve(v,e),KindV('Input',v))*16;

*restrict_extra(e)..     extra(e) =l= sum(l,Mel(e,l));

*restrict_extra(e)..     extra(e) =l= sum(v$Gve(v,e),KindV('Input',v))*1;

add(v)..                length =g= Tv(v);



cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));

obj.. cost =e= length;


*O.l(l)=0;
*
*loop(l,
*    loop((l1,l2)$Hll(l1,l2),
*        O.l(l2) = max(O.l(l1) + sum(e,Mel.l(e,l1) + Mel.l(e,l2)) -1, O.l(l2));
*    );
*);


equation block_cycles(l,l,e);
block_cycles(l1,l2,e)$Hll(l1,l2).. O(l1) + CARD(L) *(Mel(e,l1) + Mel(e,l2) -1) - CARD(L) +1 =l= O(l2);
O.up(l)=CARD(L);



Model   schedule  / all /;

solve   schedule    using mip minimizing length;
*order_length;


*schedule.optfile=1;
schedule.prioropt=1;
schedule.threads=6;
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

display Mel.l;
display Mvl.l;


display schedule.numEqu;
display schedule.etSolve;
display schedule.numVar;
 
















)";

