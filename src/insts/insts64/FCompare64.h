auto a = *reinterpret_cast<double*>(&ops[0]);
auto b = *reinterpret_cast<double*>(&ops[1]);

if (a == b) return a == (1ull << 63ull) ? 3 : 0;

if (a > b) return 1;

return 2;
