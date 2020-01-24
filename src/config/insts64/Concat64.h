// Concat 16-bit slice in cyclic order

uint32_t ret = 0x0000000000000000;

uint32_t val1 = (ops[0] & 0x00000000FFFFFFFF) << 32;
uint32_t val2 = (ops[1] & 0x00000000FFFFFFFF) << 0;

ret = val1 | val2;

return ret;
