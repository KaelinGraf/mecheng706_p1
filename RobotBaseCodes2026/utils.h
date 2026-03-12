template<typename T>
T clip(T val, T min, T max){
  if (val <= min){
    val = min;
  }else if (val >= max){
    val = max;
  }
  return val;
}