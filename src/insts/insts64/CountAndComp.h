if(accum==ops[1]) {
  accum=0; // count #ops[0]
} else {
  backpressure1 = 1;
}
accum += 1;
return (ops[0]==ops[1]);
