template <unsigned p, unsigned d>  // p为待查数字，d为当前除数
struct DoIsPrime {
  static constexpr bool value = (p % d != 0) && DoIsPrime<p, d - 1>::value;
};
template <unsigned p>
struct DoIsPrime<p, 2> {  // 除数为2，停止递归
  static constexpr bool value = (p % 2 != 0);
};
template <unsigned p>  // 基础模板
struct IsPrime {
  // 从p/2开始递归
  static constexpr bool value = DoIsPrime<p, p / 2>::value;
};
// 特殊情况（） 避免模板实例化陷入无限递归
template <>
struct IsPrime<0> {
  static constexpr bool value = false
};
template <>
struct IsPrime<1> {
  static constexpr bool value = false
};
template <>
struct IsPrime<2> {
  static constexpr bool value = true
};
template <>
struct IsPrime<3> {
  static constexpr bool value = true;
};
