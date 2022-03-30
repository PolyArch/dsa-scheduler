uint32_t mask = ~((uint16_t) 0);
int16_t a[4];
*reinterpret_cast<int64_t*>(a) = ops[0];
a[0] += a[1];
a[1] = 0;
a[2] += a[3];
a[3] = 0;
int64_t res = *reinterpret_cast<int64_t*>(a);
return res;

