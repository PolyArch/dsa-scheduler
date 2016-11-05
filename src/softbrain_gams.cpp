string softbrain_gams = R"(
$batinclude softbrain_kind.gams
$batinclude softbrain_model.gams
$batinclude softbrain_pdg.gams

file outfile / "softbrain.out" /;
outfile.pc=8;
outfile.pw=4096;
put outfile;

scalar i;

* first do the resource restriction checks


* This is the actual solve
$batinclude constraints.gams

put "[vertex-node-map]" /
loop((iv),
    put iv.tl ":";
    loop((n)$(Mn.l(iv,n)<>0),
        put n.tl
    );
    put /
);
put "[vertex-link-map]" /
loop(v,
    put v.tl ":"
    loop((l)$(Mvl.l(v,l)<>0),
        put l.tl
    );
    put /
);
*put "[extra-lat]" /
*loop(e$(extra.l(e)<>0),
*    put e.tl ":" extra.l(e) / 
*);
put "[timing]" /
loop(v,
    put v.tl ":" Tv.l(v) / 
);
put "[edge-delay]" /
loop(e,
    put e.tl ":" extra.l(e) / 
);
put "[passthrough]" /
loop(n$(PT.l(n) and sum((e,l)$(Hnl(n,l)), Mel.l(e,l))),
    put n.tl 
);
put /
put "[port-port-map]" /
loop(pv,
    put pv.tl ":";
    loop((pn)$(Mp.l(pv,pn)<>0),
        put pn.tl
    );
    for(i=1 to 32,
      loop((v,pn,n)$(Mp.l(pv,pn)<>0 and VI(pv,v)=i and Mn.l(v,n)),
        put round(PI(pn,n),1)
      );
    );
    put /
);

)";
