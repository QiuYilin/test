#pragma once
template <class K, class E>
class dictionary {
 public:
  virtual ~dictionary() {}
  /// @brief return true iff dictionary is empty
  /// @return
  virtual bool empty() const = 0;
  /// @brief return number of pairs in dictionary
  /// @return
  virtual int size() const = 0;
  /// @brief return pointer to matching pair
  /// @param
  /// @return
  virtual std::pair<const K, E>* find(const K&) const = 0;
  /// @brief remove matching pair
  /// @param
  virtual void erase(const K&) = 0;
  /// @brief insert a (key, value) pair into the dictionary
  /// @param
  virtual void insert(const pair<const K, E>&) = 0;
};