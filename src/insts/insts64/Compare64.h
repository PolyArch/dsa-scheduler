auto a = ops[0];
auto b = ops[1];

if (ops.size() == 3) {
  // G E L
  // 4 2 1
  int pred = ops[2];
  // G
  if ((pred & 4) && (a > b)) {
    return true;
  }
  // E
  if ((pred & 2) && (a == b)) {
    return true;
  }
  // L
  if ((pred & 1) && (a < b)) {
    return true;
  }
  return false;
}

if (a == b) return (a == (1ull << 63ull) ? 3 : 0);

if (a > b) return 1;

return 2;
