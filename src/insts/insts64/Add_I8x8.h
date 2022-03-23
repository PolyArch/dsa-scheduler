uint8_t a[8];
for (int i = 0; i < 8; ++i) {
  a[i] = (ops[0] >> (i * 8)) & 255;
}
uint8_t b[8];
for (int i = 0; i < 8; ++i) {
  b[i] = (ops[1] >> (i * 8)) & 255;
}
uint8_t c[8];
for (int i = 0; i < 8; ++i) {
  c[i] = a[i] + b[i];
}
return *((uint64_t*)c);
