uint32_t mask = ~((uint32_t) 0);
return (ops[0] & mask) + ((ops[0] >> 32) & mask);
