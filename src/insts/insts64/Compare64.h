auto a = ops[0];
auto b = ops[1];

if (ops.size() == 3) {
  int pred = ops[2];
  // U L G E
  if ((pred & 1) && (a == b)) {
    return true;
  }
  if ((pred & 2) && (a > b)) {
    return true;
  }
  if ((pred & 4) && (a < b)) {
    return true;
  }
  return false;
}

if (a == b) return (a == (1ull << 63ull) ? 3 : 0);

if (a > b) return 1;

return 2;
