import numpy as np
from scipy.optimize import dual_annealing

# 定义目标函数（Rastrigin函数）
def rastrigin(x):
    A = 10
    return A * len(x) + sum([(xi**2 - A * np.cos(2 * np.pi * xi)) for xi in x])

# 定义变量的边界
bounds = [(-5.12, 5.12)] * 2  # 2维变量，每个变量的范围是 [-5.12, 5.12]

# 设置初始解
x0 = np.random.uniform(-5.12, 5.12, size=2)

# 调用 dual_annealing 进行优化
result = dual_annealing(
    func=rastrigin,
    bounds=bounds,
    maxfun=1000,
    x0=x0,
    no_local_search=False,
    minimizer_kwargs={
        'method': 'SLSQP',
        'options': {'maxiter': 100}
    }
)

# 打印优化结果
print("Optimal solution:", result.x)
print("Objective function value at optimal solution:", result.fun)
print("Optimization result message:", result.message)