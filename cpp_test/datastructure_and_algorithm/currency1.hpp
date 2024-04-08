#include <iostream>
class illegalParameterValue {
 public:
  illegalParameterValue() : message("Illegal parameter value") {}

  illegalParameterValue(char* theMessage) { message = theMessage; }

  void outputMessage() { std::cout << message << std::endl; }

 private:
  std::string message;
};

enum signType { plus, minus };

class currency {
 public:
  currency(signType theSign = plus, unsigned long theDollars = 0,
           unsigned int theCents = 0);
  ~currency() {}
  void setValue(signType, unsigned long, unsigned int);
  void setValue(double);
  signType getSign() const { return sign; }
  unsigned long getDollars() const { return dollars; }
  unsigned long getCents() const { return cents; }
  currency add(const currency&) const;   // 返回局部对象所以返回值
  currency& increment(const currency&);  // 返回调用对象所以用引用
  void output() const;

 private:
  signType sign;
  unsigned long dollars;
  unsigned int cents;
};

currency::currency(signType theSign, unsigned long theDollars,
                   unsigned int theCents) {
  setValue(theSign, theDollars, theCents);
}

void currency::setValue(signType theSign, unsigned long theDollars,
                        unsigned int theCents) {
  if (theCents > 99) {
    throw illegalParameterValue();  // 所以抛出的异常只需要是一个类
  }
  sign = theSign;
  dollars = theDollars;
  cents = theCents;
}

void currency::setValue(double theAmount) {
  if (theAmount < 0) {
    sign = minus;
    theAmount = -theAmount;
  } else {
    sign = plus;
  }
  dollars = (unsigned long)theAmount;
  cents = (unsigned int)((theAmount + 0.001 - dollars) * 100);
}

currency currency::add(const currency& x) const {
  long a1, a2, a3;
  currency result;
  a1 = dollars * 100 + cents;
  if (sign == minus) {
    a1 = -a1;
  }

  a2 = x.dollars * 100 + x.cents;
  if (x.sign == minus) {
    a2 = -a2;
  }

  a3 = a1 + a2;

  if (a3 < 0) {
    result.sign = minus;
    a3 = -a3;
  } else {
    result.sign = plus;
  }
  result.dollars = a3 / 100;
  result.cents = a3 - result.dollars * 100;
  return result;
}

currency& currency::increment(const currency& x) {
  *this = add(x);
  return *this;
}

void currency::output() const {
  if (sign == minus) {
    std::cout << '-';
  }
  if (cents < 10) {
    std::cout << '0';
  }
  std::cout << cents;
}