uint16_t a0 = (ops[0] & 0x000000000000FFFF) >> 0;
uint16_t a1 = (ops[0] & 0x00000000FFFF0000) >> 16;
uint16_t a2 = (ops[0] & 0x0000FFFF00000000) >> 32;
uint16_t a3 = (ops[0] & 0xFFFF000000000000) >> 48;
uint32_t b0 = (reg[0] & 0x00000000FFFFFFFF) >> 0;
uint32_t b1 = (reg[0] & 0xFFFFFFFF00000000) >> 32;
uint32_t b2 = (reg[1] & 0x00000000FFFFFFFF) >> 0;
uint32_t b3 = (reg[1] & 0xFFFFFFFF00000000) >> 32;

b0 += a0;
b1 += a1;
b2 += a2;
b3 += a3;

uint64_t c0 = (uint64_t)(b0) << 0;
uint64_t c1 = (uint64_t)(b1) << 32;
uint64_t c2 = (uint64_t)(b2) << 0;
uint64_t c3 = (uint64_t)(b3) << 32;

reg[0] = c0 | c1;
reg[1] = c2 | c3;

outs[1] = reg[1];
return reg[0];
