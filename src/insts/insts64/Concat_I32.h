// Concat 16-bit slice in cyclic order


uint64_t val1 = (ops[0] & 0x000000000000FFFF) << 0;
uint64_t val2 = (ops[1] & 0x00000000FFFF0000) << 16;

return val1 | val2;
