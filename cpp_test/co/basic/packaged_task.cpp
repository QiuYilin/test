#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

std::mutex m;
std::deque<std::packaged_task<void()> > tasks;

bool gui_shutdown_message_received();
void get_and_process_gui_message();

void gui_thread() {  // 图形线程
  while (
      !gui_shutdown_message_received()) {  // 一直循环直到收到关闭界面的信息然后关闭界面
    get_and_process_gui_message();  // 关闭界面前轮询界面消息处理消息
    std::packaged_task<void()> task;
    {
      std::lock_guard<std::mutex> lk(m);
      if (tasks.empty()) continue;      // r如果 没有任务就继续循环
      task = std::move(tasks.front());  // 提取出一个任务
      tasks.pop_front();
    }
    task();  // 执行任务
  }
}

std::thread gui_bg_thread(gui_thread);

template <typename Func>
std::future<void> post_task_for_gui_thread(Func f) {
  std::packaged_task<void()> task(f);         // 提供一个打包好的任务
  std::future<void> res = task.get_future();  // 获取future对象
  std::lock_guard<std::mutex> lk(m);
  tasks.push_back(std::move(task));
  return res;
}
