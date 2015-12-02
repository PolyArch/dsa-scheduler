string dyser_gams = R"(
$batinclude dyser_kind.gams
$batinclude dyser_model.gams
$batinclude dyser_pdg.gams
$batinclude constraints.gams

file outfile / "dyser.out" /;
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
put "[vertex-link-map]" /
loop(v,
    put v.tl ":"
    loop((l)$(Mvl.l(v,l)<>0),
        put l.tl
    );
    put /
);
put "[extra-lat]" /
loop(e$(extra.l(e)<>0),
    put e.tl ":" extra.l(e) / 
);
put "[port-port-map]" /
loop(pv,
    put pv.tl ":";
    loop((pn)$(Mp.l(pv,pn)<>0),
        put pn.tl
    );
    put /
);
)";
