$batinclude softbrain_kind.gams
$batinclude softbrain_model.gams
$batinclude softbrain_pdg.gams
$batinclude constraints.gams

*file outfile / "softbrain.out" /;
*outfile.pc=8;
*outfile.pw=4096;
*outfile.ap=1;
put outfile;

scalar i;


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
put "[switch-map]" /
loop(r,
    put r.tl ":"
    loop((l1,l2)$(Hlr(l1,r) and Hrl(r,l2) and Sll.l(l1,l2)<>0),
        put l1.tl l2.tl","
    );
    put /
);
put "[timing]" /
loop(v,
    put v.tl ":" Tv.l(v) / 
);
*put "[link-order]" /
*loop(l,
*    put l.tl ":" O.l(l) / 
*);
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
