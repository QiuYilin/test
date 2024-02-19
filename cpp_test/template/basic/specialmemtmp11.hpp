#pragma once
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

class Person {
 private:
  std::string name;

 public:
  // 传递初始化成员name的构造函数
  explicit Person(std::string const& n) : name(n) {
    std::cout << "copying string-CONSTR for '" << name << "'\n";
  }
  explicit Person(std::string&& n) : name(std::move(n)) {
    std::cout << "moving string-CONSTR for '" << name << "'\n";
  }

  Person(Person const& p) : name(p.name) {
    std::cout << "COPY-CONSTR Person '" << name << "'\n";
  }

  Person(Person&& p) : name(std::move(p.name)) {
    std::cout << "MOVE-CONSTR Person '" << name << "'\n";
  }
};

class Person_ {
 private:
  std::string name;

 public:
  template <typename STR>
  explicit Person_(STR&& n) : name(std::forward<STR>(n)) {
    std::cout << "tmpl-CONSTR for '" << name << "'\n";
  }
  Person_(Person_ const& p) : name(p.name) {
    std::cout << "COPY-CONSTR Person '" << name << "'\n";
  }
  Person_(Person_&& p) : name(std::move(p.name)) {
    std::cout << "MOVE-CONSTR Person '" << name << "'\n";
  }
};

template <typename T>
using EnableIfString = std::enable_if_t<std::is_convertible_v<T, std::string>>;

class PersonT {
 private:
  std::string name;

 public:
  template <typename STR, typename = EnableIfString<STR>>
  explicit PersonT(STR&& n) : name(std::forward<STR>(n)) {
    std::cout << "tmpl-CONSTR for '" << name << "'\n";
  }
  PersonT(PersonT const& p) : name(p.name) {
    std::cout << "COPY-CONSTR Person '" << name << "'\n";
  }
  PersonT(PersonT&& p) : name(std::move(p.name)) {
    std::cout << "MOVE-CONSTR Person '" << name << "'\n";
  }
};