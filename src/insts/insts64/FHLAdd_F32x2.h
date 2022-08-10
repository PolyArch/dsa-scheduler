uint32_t mask = ~((uint32_t) 0);

return as_float(ops[0] & mask) + as_float((ops[0] >> 32) & mask);
