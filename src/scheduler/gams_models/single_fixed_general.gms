Variable            cost,length;
binary variable     Mn(v,n), Ml(e,l), Mvl(v,l);

positive variable   Tv(v), Te(e),invThroughput;

alias(v1,v2,v);
alias (l1,l2,l);
set Gvv(v,v);
Gvv(v1,v2)=YES$(sum(e,Gve(v1,e) and Gev(e,v2))); 
set Hll(l,l);
Hll(l1,l2)=YES$( sum(n,Hln(l1,n) and Hnl(n,l2)) or (sum(r,Hlr(l1,r) and Hrl(r,l2)) and not sum(n,Hnl(n,l1) and Hln(l2,n))) );

positive variable   O(l),extra(e),maxExtra;

$batinclude mip_start.gams
alias (v1,v2,v);

Equations
    assignVertex(K,v)
    oneVperN(n)
    oneEperL(l)
    add(v)
    source_mapping(e,n)
    incoming_links(e,r)
    outgoing_links(e,r)
*    latency(e,v)
*    latency2(e,v)
    obj;

Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
Ml.prior(e,l)=10;
Mvl.prior(v,l)=100;

* Set not-possible variables to 0
loop(K,
Mn.fx(v,n)$(kindV(K,v) and not kindN(K,n))=0;);
*loop((K,v,n),
*Ml.fx(e,l)$(Gve(v,e) and Hnl(n,l) and kindV(K,v) and not kindN(K,n) )=0;);
*loop((K,n),
*Mvl.fx(v,l)$(Hnl(n,l) and kindV(K,v) and not kindN(K,n) )=0;);

*Mn.fx(inV,n)=0;
*Mn.fx(outV,n)=0;

* Set input latencies
Tv.fx(v)$kindV('Input',v)=0;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =e= 1;
oneVperN(n)..                      sum(v,Mn(v,n)) =l= 1;

*equation calc_l_used(v,l);
*calc_l_used(v,l)..                 sum(e$(Gve(v,e)),Ml(e,l)) - 25*Mvl(v,l) =l= 0;

equation opposite_calc_l_used(v,l);
opposite_calc_l_used(v,l)..  sum(e$(Gve(v,e)),Ml(e,l)) =g= Mvl(v,l);

equation calc_l_used2(v,e,l);
calc_l_used2(v,e,l)$(Gve(v,e))..   Ml(e,l) =l= Mvl(v,l);

oneEperL(l)..                  sum(v,Mvl(v,l)) =l= 1;

*don't map inputs to outputs
*$((not KindN('Output',n)) and (not KindN('Input',n)))
source_mapping(e,n)..
                        sum(l$Hnl(n,l),Ml(e,l)) =e= sum(v$Gve(v,e), Mn(v,n));

*equation dest_mapping0(e,n,v,l);                        
*dest_mapping0(e,n,v,l)$((not KindN('Output',n)) and (not KindN('Input',n)) and Gev(e,v) and Hln(l,n)).. 
*                        Ml(e,l) =l= Mn(v,n);
               
*and (not KindN('Input',n))(not KindN('Output',n)) ) and
               
equation dest_mapping(e,n,v);
dest_mapping(e,n,v)$( Gev(e,v)).. 
                        sum(l$Hln(l,n),Ml(e,l)) =e= Mn(v,n);
*              sum((l,v)$(Hln(l,n) and Gve(v,e)),Mvl(v,l)) =e= sum(v$Gev(e,v), Mn(v,n));

*equation dest_mapping2(e,n,l,v);                        
*dest_mapping2(e,n,l,v)$((not KindN('Output',n)) and (not KindN('Input',n)) and Hln(l,n) and Gev(e,v)).. 
*                        Mvl(v,l) =e= Mn(v,n);
                        
*Ml(e,l) =e= sum(v$Gev(e,v), Mn(v,n));

incoming_links(e,r)..   sum(l$Hlr(l,r),Ml(e,l)) =e= sum(l$Hrl(r,l), Ml(e,l));
outgoing_links(e,r)..   sum(l$Hlr(l,r),Ml(e,l)) =l= 1;

*experimental
equation limit_inc_v(v,r);
limit_inc_v(v,r)..   sum(l$Hlr(l,r),Mvl(v,l)) =l= 1;

*equation io_mapping2(e,n);
*io_mapping2(intedges,n)$(KindN('Input',n) or KindN('Output',n))..
*                        sum(l$Hln(l,n),Ml(intedges,l)) =l= 1;                        

*latency(e,v)$Gve(v,e)..         Te(e) =e= Tv(v) + sum(l,Ml(e,l)) + delta(e);
*latency2(e,v)$Gev(e,v)..        Tv(v) =g= Te(e);

equation latency(v,e,v);
latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     Tv(v2) =e= Tv(v1) + sum(l,Ml(e,l)) + delta(e) + extra(e);


*positive varaible Tn(n), Tr(r), Tl(l);

*latency3(e,n,l)$(Hnl(n,l)).. Tl(l) =e= Tn(n) + 1;
*latency4(r,l)$(Hl(n,l)).. Tl(l) =e= Tn(n) + 1;

add(v)..                length =g= Tv(v);


*cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l;
cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));


obj.. cost =e= length;
*obj.. cost =e=  1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) +  1000 * length + sum(l,sum(v,Mvl(v,l)));

*obj.. cost =e=  1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) + 1000 * length;
*obj.. cost =e=  100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) +100000*maxExtra+  1000 * length + sum(l,sum(v,Mvl(v,l)));
*obj.. cost =e=  100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) + 1000 * length;

equation getMismatch(e);
getMismatch(e).. maxExtra =g= extra(e);

Model   schedule  / all /;

schedule.optfile=1;
schedule.prioropt=1;
schedule.threads=32;
*schedule.reslim=100;
*schedule.holdFixed=1;

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

*schedule.Cutoff = 1000000;


solve   schedule    using mip minimizing cost;


equation setLen;
setLen.. length =l= length.l;

variable cost2;

O.l(l)=0;

loop(l,
    loop((l1,l2)$Hll(l1,l2),
        O.l(l2) = max(O.l(l1) + sum(e,Ml.l(e,l1) + Ml.l(e,l2)) -1, O.l(l2));
    );
);

equation block_cycles(l,l,e);
block_cycles(l1,l2,e)$Hll(l1,l2).. O(l1) + Ml(e,l1) + Ml(e,l2) -1 =l= O(l2);



*cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l;
*cost2.l = 100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));
cost2.l = 100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) + maxExtra.l;


equation obj2;
obj2.. cost2 =e=  100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) + maxExtra;
*obj2.. cost2 =e=  100000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) + 1000 * length;

O.up(l)=CARD(L);

Model   schedule2  / all /;

schedule2.optfile=1;
schedule2.prioropt=1;
schedule2.threads=32;

schedule2.reslim=1060;
schedule2.optCA=0.999;
*solve   schedule2    using mip minimizing cost2;

display extra.l;
display maxExtra.l;

display schedule.numEqu;
display schedule2.numEqu;

display schedule.etSolve;
display schedule2.etSolve;

display schedule.numVar;
display schedule2.numVar;
