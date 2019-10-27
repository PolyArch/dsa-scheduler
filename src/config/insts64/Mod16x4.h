uint16_t a0 = (ops[0] & 0x000000000000FFFF) >> 0;
uint16_t a1 = (ops[0] & 0x00000000FFFF0000) >> 16;
uint16_t a2 = (ops[0] & 0x0000FFFF00000000) >> 32;
uint16_t a3 = (ops[0] & 0xFFFF000000000000) >> 48;
uint16_t b0 = (ops[1] & 0x000000000000FFFF) >> 0;
uint16_t b1 = (ops[1] & 0x00000000FFFF0000) >> 16;
uint16_t b2 = (ops[1] & 0x0000FFFF00000000) >> 32;
uint16_t b3 = (ops[1] & 0xFFFF000000000000) >> 48;
a0 %= b0;
a1 %= b1;
a2 %= b2;
a3 %= b3;
uint64_t c0 = (uint64_t)(a0) << 0;
uint64_t c1 = (uint64_t)(a1) << 16;
uint64_t c2 = (uint64_t)(a2) << 32;
uint64_t c3 = (uint64_t)(a3) << 48;
return c0 | c1 | c2 | c3;
