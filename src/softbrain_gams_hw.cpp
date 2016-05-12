string softbrain_gams_hw = R"(
$batinclude softbrain_kind.gams
$batinclude softbrain_model.gams
$batinclude softbrain_pdg.gams
$batinclude constraints.gams

file outfile / "softbrain.out" /;
outfile.pc=8;
outfile.pw=4096;
put outfile;

put "[vertex-node-map]" /
loop((iv),
    put iv.tl ":";
    loop((n)$(Mn.l(iv,n)<>0),
        put n.tl
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
put "[port-port-map]" /
loop(pv,
    put pv.tl ":";
    loop((pn)$(Mp.l(pv,pn)<>0),
        put pn.tl
    );
    put /
);
put "[dfu-lat]" /
loop(n$(En.l(n)<>0),
    put n.tl ":" En.l(n) / 
);

put "[extra-lat]" /
loop(e$(extra.l(e)<>0),
    put e.tl ":" extra.l(e) / 
);


*put "[vertex-link-map]" /
*loop(v,
*    put v.tl ":"
*    loop((l)$(Mvl.l(v,l)<>0),
*        put l.tl
*    );
*    put /
*);
)";
