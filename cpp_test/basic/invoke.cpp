#include <functional>  //std::forward所需的头文件
#include <utility>     //std::invoke所需头文件

template <typename Callable, typename... Args>
decltype(auto) call(Callable&& op, Args&&... args) {
  if constexpr (std::is_same_v<std::invoke_result_t<Callable, Args...>, void>) {
    std::invoke(std::forward<Callable>(op), std::forward<Args>(args)...) return;
  } else {
    return std::invoke(std::forward<Callable>(op), std::forward<Args>(args)...);
  }
}
