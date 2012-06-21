string spill_model = R"(


Variable            cost,length;
binary variable     Mn(v,n), Ml(e,l), Mvl(v,l)
positive variable    Tv(v), Te(e);

Equations
    assignVertex(K,v)
    oneVperN(n)
    oneEperL(l)
    add(v)
    source_mapping(e,n)
    dest_mapping(e,n)
    incoming_links(e,r)
    outgoing_links(e,r)
    latency(e,v)
    latency2(e,v)
    obj;

Mn.prior(v,n)=0;
Mn.prior(v,n)$(kindV('Input',v) or kindN('Input',n))=5;
Mn.prior(v,n)$(kindV('Output',v) or kindN('Output',n))=30;
Ml.prior(e,l)=10;
Mvl.prior(v,l)=100;

* Set not-possible variables to 0
loop(K,
Mn.fx(v,n)$(kindV(K,v) and not kindN(K,n))=0;);
loop((K,v,n),
Ml.fx(e,l)$(Gve(v,e) and Hnl(n,l) and kindV(K,v) and not kindN(K,n) )=0;);
loop((K,n),
Mvl.fx(v,l)$(Hnl(n,l) and kindV(K,v) and not kindN(K,n) )=0;);

* Set input latencies
Tv.fx(v)$kindV('Input',v)=0;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =l= 1;
oneVperN(n)..                      sum(v,Mn(v,n)) =l= 1;

equation calc_l_used(v,l);
calc_l_used(v,l)..                 sum(e$(Gve(v,e)),Ml(e,l)) - 25*Mvl(v,l) =l= 0;

*equation calc_l_used(v,e,l);
*calc_l_used(v,e,l)$(Gve(v,e))..   Ml(e,l) =l= Mvl(v,l);

oneEperL(l)..                  sum(v,Mvl(v,l)) =l= 1;

source_mapping(e,n)..                      sum(l$Hnl(n,l),Ml(e,l)) =e= sum(v$Gve(v,e), Mn(v,n));
dest_mapping(e,n)$(not KindN('Output',n)).. sum(l$Hln(l,n),Ml(e,l)) =e= sum(v$Gev(e,v), Mn(v,n));
incoming_links(e,r)..                      sum(l$Hlr(l,r),Ml(e,l)) =e= sum(l$Hrl(r,l), Ml(e,l));
outgoing_links(e,r)..                      sum(l$Hlr(l,r),Ml(e,l)) =l= 1;

*latency(e)..            Te(e) =e= sum(v$Gve(v,e), Tv(v)) + delta(e) + sum(l,Ml(e,l));
*latency2(e)..           Te(e) =l= sum(v$Gev(e,v), Tv(v));
latency(e,v)$Gve(v,e)..         Te(e) =e= Tv(v) + sum(l,Ml(e,l)) + delta(e);
latency2(e,v)$Gev(e,v)..        Tv(v) =g= Te(e);

add(v)..                length =g= Tv(v);

*obj.. cost =e= (1000 * S) + length;
*obj.. cost =e=  1000000* sum((v,k)$kindV(K,v),(1-sum(n$(kindN(K,n)), Mn(v, n)))) +  1000 * length + sum(l,sum(v,Mvl(v,l)));
obj.. cost =e=  1000000* sum((v,k)$kindV(K,v),(1-sum(n$(kindN(K,n)), Mn(v, n)))) + 1000 * length + sum(l,sum(v,Mvl(v,l)));

option reslim=100; 
option optcr=.1;  
option optca=999; 

Model   schedule  / all /;

schedule.optfile=1;
schedule.prioropt=1;
schedule.threads=32;
schedule.reslim=100;
*schedule.holdFixed=1;

file optfile /cplex.opt/;      

put optfile;
*put 'mipemphasis 2'/;
put 'parallelmode -1'/;
put 'probe 2'/;
*put 'heurfreq 5'/;
*put 'coeredind 3'/;
*put 'mipstart 3'/;
*put 'lpmethod 3'/;
*put 'mipsearch 1'/;
*put 'bndstrenind 1'/;
*put 'symmetry 5'/;
*put 'varsel 3'/;
*put 'depind 3'/;
*put 'divetype 3'/;
*put 'cuts 3'/;
*put 'lbheur 1'/;
*put 'mipordtype 3'/;
*put 'ppriind 2'/;
*put 'nodesel 3'/;
*put 'subalg 2'/;
*put 'mipordtype 3'/;
putclose;

solve   schedule    using mip minimizing cost;

)";    