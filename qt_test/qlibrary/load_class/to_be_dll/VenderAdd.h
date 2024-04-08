#pragma once
#include "Interface.h"

class VenderAdd : public Interface{
public:
  VenderAdd(){}
  ~VenderAdd(){}

  int Add(int ele){ _sum += ele; return _sum;}
  void SetSum(int sum){ _sum = sum;}
  int GetSum(){return _sum;}

private:
  int _sum = 0;
};