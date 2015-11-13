string dyser_gams_hw = R"(
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
put "[switch-map]" /
loop(r,
    put r.tl ":"
    loop((l1,l2)$(Hlr(l1,r) and Hrl(r,l2) and Sll.l(l1,l2)<>0),
        put l1.tl l2.tl","
    );
    put /
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
