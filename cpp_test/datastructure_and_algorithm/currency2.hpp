#include <iostream>

enum signType { plus, minus };

class illegalParameterValue {
 public:
  illegalParameterValue() : message("Illegal parameter value") {}

  illegalParameterValue(char* theMessage) { message = theMessage; }

  void outputMessage() { std::cout << message << std::endl; }

 private:
  std::string message;
};

class currency {
 public:
  currency(signType theSign = plus, unsigned long theDollars = 0,
           unsigned int theCents = 0);

  ~currency() {}
  void setValue(signType, unsigned long, unsigned int);
  void setValue(double);

  signType getSigen() const {
    if (amount < 0) {
      return minus;
    } else {
      return plus;
    }
  }

  unsigned long getDollars() const {
    if (amount < 0) {
      return (-amount) / 100;
    } else {
      return amount / 100;
    }
  }

  unsigned int getCents() const {
    if (amount < 0) {
      return -amount - getDollars() * 100;
    } else {
      return amount - getDollars() * 100;
    }
  }

  currency add(const currency&) const;
  currency& increment(const currency& x) {
    amount += x.amount;
    return *this;
  }

  currency operator+(const currency&) const;
  currency operator+=(const currency& x) {
    amount += x.amount;
    return *this;
  }

  void output(std::ostream&) const;

 private:
  long amount;
};

currency::currency(signType theSign, unsigned long theDollars,
                   unsigned int theCents) {
  setValue(theSign, theDollars, theCents);
}

void currency::setValue(signType theSign, unsigned long theDollars,
                        unsigned int theCents) {
  if (theCents > 99) {
    throw illegalParameterValue("Cents should be < 100");
  }

  amount = theDollars * 100 + theCents;

  if (theSign == minus) {
    amount = -amount;
  }
}

void currency::setValue(double theAmount) {
  if (theAmount < 0) {
    amount = (long)((theAmount - 0.001) * 100);
  } else {
    amount = (long)(theAmount + 0.001 * 100);
  }
}

currency currency::add(const currency& x) const {
  currency y;
  y.amount = amount + x.amount;
  return y;
}

currency currency::operator+(const currency& x) const {
  currency result;
  result.amount = amount + x.amount;
  return result;
}

void currency::output() const {
  long theAmount = amount;
  if (theAmount < 0) {
    std::cout << '-';
    theAmount = -theAmount;
  }
  long dollars = theAmount / 100;
  std::cout << "$" << dollars << ".";
  int cents = theAmount - dollars * 100;
  if (cents < 10) {
    std::cout << '0';
  }
  std::cout << cents;
}

void currency::output(std::ostream& out) const {
  long theAmount = amount;
  if (theAmount < 0) {
    std::cout << '-';
    theAmount = -theAmount;
  }
  long dollars = theAmount / 100;
  std::cout << "$" << dollars << ".";
  int cents = theAmount - dollars * 100;
  if (cents < 10) {
    std::cout << '0';
  }
  std::cout << cents;
}

std::ostream& operator<<(std::ostream& out, const currency& x) {
  x.output(out);
  return out;
}