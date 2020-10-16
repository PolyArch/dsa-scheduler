auto a = ops[0];
auto b = ops[1];

if (a == b) return a == (1ull << 15ull) ? 3 : 0;

if (a > b) return 1;

return 2;
