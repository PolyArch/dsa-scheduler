uint32_t mask = ~((uint32_t) 0);
return (ops[0] & mask) | ((ops[1] & mask) << 32);
