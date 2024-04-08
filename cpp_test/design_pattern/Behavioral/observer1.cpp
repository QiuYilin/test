#include <any>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

struct Subject;

struct Observer {
  virtual ~Observer() = default;
  virtual void subject_changed(
      Subject& p, const std::string& property_name,
      const std::any new_value) = 0;  // 用std::any代替了模板实现
};

struct Subject {
  explicit Subject(int state1) : state1(state1) {}

  int get_state1() const { return state1; }

  void set_state1(const int state1);

  bool get_qualification() const { return state1 >= 16; }

  void subscribe(Observer* pl);

  void unsubscribe(Observer* pl);

  void notify(const std::string& property_name, const std::any new_value);

 private:
  std::mutex mtx;
  int state1;
  std::vector<Observer*> listeners;
};

void Subject::set_state1(const int state1) {
  if (this->state1 == state1) return;

  auto old_c_v = get_qualification();

  this->state1 = state1;
  notify("state1", this->state1);

  auto new_c_v = get_qualification();
  if (old_c_v != new_c_v) {
    notify("can_vote", new_c_v);
  }
}

void Subject::subscribe(Observer* pl) {
  std::lock_guard<std::mutex> lock(mtx);
  listeners.push_back(pl);
}

void Subject::unsubscribe(Observer* pl) {
  std::lock_guard<std::mutex> lock(mtx);
  listeners.erase(std::remove(listeners.begin(), listeners.end(), pl),
                  listeners.end());
}  // Erase-Remove Idiom

void Subject::notify(const std::string& property_name,
                     const std::any new_value) {
  std::lock_guard<std::mutex> lock(mtx);
  for (const auto listener : listeners)
    listener->subject_changed(*this, property_name, new_value);
}

struct ConcreteObserver : Observer {
  void subject_changed(Subject& p, const std::string& property_name,
                       const std::any new_value) override {
    std::cout << "subject's " << property_name << " has been changed to ";
    if (property_name == "state1") {
      std::cout << std::any_cast<int>(new_value);
    } else if (property_name == "can_vote") {
      std::cout << std::any_cast<bool>(new_value);
    }
    std::cout << "\n";
  }
};

int main() {
  Subject p{14};
  ConcreteObserver cl;
  p.subscribe(&cl);
  p.set_state1(15);
  p.set_state1(16);

  getchar();
  return 0;
}
