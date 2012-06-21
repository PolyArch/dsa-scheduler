string multi_model = R"(

Variable            cost,length;
binary variable     Mn(v,n), Ml(e,l), Mvl(v,l);
positive variable   Tv(v), Te(e),invThroughput;
alias(v1,v2,v);

$batinclude mip_start.gams

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

Mn.fx(inV,n)=0;
Mn.fx(outV,n)=0;

* Set input latencies
Tv.fx(v)$kindV('Input',v)=0;

assignVertex(K,v)$kindV(K,v)..     sum(n$(kindN(K,n)), Mn(v, n)) =l= 1;
oneVperN(n)..                      sum(v,Mn(v,n)) =l= 1;

equation calc_l_used(v,l);
calc_l_used(v,l)..                 sum(e$(Gve(v,e)),Ml(e,l)) - 25*Mvl(v,l) =l= 0;

*equation calc_l_used2(v,e,l);
*calc_l_used2(v,e,l)$(Gve(v,e))..   Ml(e,l) =l= Mvl(v,l);

oneEperL(l)..                  sum(v,Mvl(v,l)) =l= 1;

*don't map inputs to outputs
source_mapping(e,n)$((not KindN('Output',n)) and (not KindN('Input',n)))..
                        sum(l$Hnl(n,l),Ml(e,l)) =e= sum(v$Gve(v,e), Mn(v,n));

*equation dest_mapping0(e,n,v,l);                        
*dest_mapping0(e,n,v,l)$((not KindN('Output',n)) and (not KindN('Input',n)) and Gev(e,v) and Hln(l,n)).. 
*                        Ml(e,l) =l= Mn(v,n);
                        
equation dest_mapping(e,n,v);                        
dest_mapping(e,n,v)$((not KindN('Output',n)) and (not KindN('Input',n)) and Gev(e,v)).. 
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

* don't let non-i/o links be stashed to i/o nodes
equation out_mapping(e,n);
out_mapping(e,n)$(KindN('Output',n))..
                        sum(l$Hln(l,n),Ml(e,l)) =e= sum(l$Hnl(n,l), Ml(e,l));

equation in_mapping(e,n);
in_mapping(e,n)$(KindN('Input',n))..
                        sum(l$Hln(l,n),Ml(e,l)) =e= sum(l$Hnl(n,l), Ml(e,l));

* should this be per vertex or per edge                        
equation one_loadslice(v);
one_loadslice(v).. sum(loadlinks,Mvl(v,loadlinks)) =l= 1;
                        
*equation io_mapping2(e,n);
*io_mapping2(intedges,n)$(KindN('Input',n) or KindN('Output',n))..
*                        sum(l$Hln(l,n),Ml(intedges,l)) =l= 1;                        

*latency(e,v)$Gve(v,e)..         Te(e) =e= Tv(v) + sum(l,Ml(e,l)) + delta(e);
*latency2(e,v)$Gev(e,v)..        Tv(v) =g= Te(e);

equation latency(v,e,v);
latency(v1,e,v2)$(Gve(v1,e) and Gev(e,v2))..     Tv(v2) =g= Tv(v1) + sum(l,Ml(e,l)) + delta(e);

*positive varaible Tn(n), Tr(r), Tl(l);

*latency3(e,n,l)$(Hnl(n,l)).. Tl(l) =e= Tn(n) + 1;
*latency4(r,l)$(Hl(n,l)).. Tl(l) =e= Tn(n) + 1;

add(v)..                length =g= Tv(v);

*obj.. cost =e= (1000 * S) + length;
obj.. cost =e=  1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) +  1000 * length + sum(l,sum(v,Mvl(v,l)));
*obj.. cost =e=  1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn(iv, n)))) + 1000 * length;

Model   schedule  / all /;

schedule.optfile=1;
schedule.prioropt=1;
schedule.threads=32;
*schedule.reslim=100;
*schedule.holdFixed=1;

file optfile /cplex.opt/;      

put optfile;
*put 'mipemphasis 2'/;
*put 'parallelmode -1'/;
*put 'probe 3'/;
*put 'heurfreq 1'/;
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
  
)";
