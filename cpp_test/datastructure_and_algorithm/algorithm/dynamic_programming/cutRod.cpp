#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// 递归方法
int cutRodRecurse(const std::vector<int>& prices, int n) {
  if (n == 0) {
    return 0;
  }
  int q = 0;
  for (int i = 1; i <= n; i++) {
    q = std::max(
        q,
        prices[i] + cutRodRecurse(prices, n - i));  // 数学公式和代码的等价写法
  }
  return q;
}

int meimoized_cut_rod_aux(const std::vector<int>& prices, int n,
                          std::vector<int> r) {
  int q;
  if (r[n] >= 0) {
    return r[n];
  }
  if (n == 0) {
    q = 0;
  } else {
    q = INT_MIN;
    for (int i = 1; i < n + 1; i++) {
      q = std::max(q, prices[i] + meimoized_cut_rod_aux(prices, n - i, r));
    }
  }
  r[n] = q;
  return q;
}

// 带备忘录的动态规划
int memoized_cut_rod(const std::vector<int>& prices, int n) {
  std::vector<int> memo(n + 1, -1);  // 保存中间结果的数组
  return meimoized_cut_rod_aux(prices, n, memo);
}

// 动态规划解法
std::vector<int> bottom_up_cut_rod(vector<int>& prices, int n, int& profit) {
  std::vector<int> r(n + 1);
  r[0] = 0;
  std::vector<int> s(n + 1);
  for (int j = 1; j <= n; j++) {  // 逐个求解规模为j的子问题
    int q = INT_MIN;
    for (int i = 1; i <= j; i++) {
      // q = std::max(q, prices[i] + r[j -
      // i]);//直接通过元素r[j-i]来获得规模为j-i的子问题的解
      if (q < prices[i] + r[j - i]) {
        q = prices[i] + r[j - i];
        s[j] = i;  // 保存对应的第一段的切割长度
      }
    }
    r[j] = q;  // 将规模为j的子问题的解存入r[j]
  }
  profit = r[n];
  return s;
}

#include <chrono>

int main() {
  vector<int> prices = {0,  1,  5,  8,  9,  10, 17,
                        17, 20, 24, 30, 32, 37};  // 钢条长度对应的价格
  int n = 4;                                      // 钢条的总长度
  int maxProfit;

  auto start = std::chrono::high_resolution_clock::now();
  maxProfit = cutRodRecurse(prices, n);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  cout << "max profit:" << maxProfit << endl;
  cout << "Execution time for cutRodRecurse: " << duration.count()
       << " microseconds" << endl;

  start = std::chrono::high_resolution_clock::now();
  maxProfit = memoized_cut_rod(prices, n);
  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  cout << "max profit:" << maxProfit << endl;
  cout << "Execution time for cutRodRecurse: " << duration.count()
       << " microseconds" << endl;

  start = std::chrono::high_resolution_clock::now();

  std::vector<int> solution = bottom_up_cut_rod(prices, n, maxProfit);
  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  cout << "max profit:" << maxProfit << endl;
  cout << "Execution time for cutRod: " << duration.count() << " microseconds"
       << endl;
  int k = n;
  while (k > 0) {
    std::cout << "" << solution[k];
    k = k - solution[k];
  }

  return 0;
}
