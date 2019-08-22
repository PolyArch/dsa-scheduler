if(ops[1]==0) {
    outs[1] = 0; //secondary output (REM)
    return 0;
}

outs[1] = ops[0]%ops[1]; //rem
return ops[0]/ops[1];
