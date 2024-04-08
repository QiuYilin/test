#pragma once

class Interface{
public:
  virtual int Add(int ele) = 0;
  virtual void SetSum(int sum) = 0;
  virtual int GetSum() =0;
};