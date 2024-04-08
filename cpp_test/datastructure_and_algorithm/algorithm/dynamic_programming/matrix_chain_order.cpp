#include <iostream>
#include <vector>

int matrixChainOrder(std::vector<int>& dimensions) {
  int n = dimensions.size() - 1;
  std::vector<std::vector<int>> dp(n, std::vector<int>(n, 0));  // 保存代价的表

  for (int len = 2; len <= n; ++len) {  // 链长度
    for (int i = 0; i < n - len + 1; ++i) {
      int j = i + len - 1;
      dp[i][j] = INT_MAX;
      // 在以上表达出i,j   Ai,j是A1n中的任意一个矩阵
      for (int k = i; k < j; ++k) {
        int cost = dp[i][k] + dp[k + 1][j] +
                   dimensions[i] * dimensions[k + 1] *
                       dimensions[j + 1];  // 状态转移方程
        if (cost < dp[i][j]) {
          dp[i][j] = cost;
        }
      }
    }
  }

  return dp[0][n - 1];
}

int main() {
  std::vector<int> dimensions = {10, 30, 5, 60};
  int minCost = matrixChainOrder(dimensions);
  std::cout << "Minimum number of multiplications: " << minCost << std::endl;

  return 0;
}
