double result = *reinterpret_cast<double*>(&accum) + *reinterpret_cast<double*>(&ops[0]);

accum = *reinterpret_cast<uint64_t*>(&result);

return accum;
