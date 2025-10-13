import casadi as ca
import numpy as np

# 定义时间变量 t
t = ca.SX.sym('t')

# 定义时间变化矩阵 A(t)
A0 = np.array([[0, 1], [-2, -3]])
A1 = np.array([[0, 0], [0, -1]])

# 定义矩阵 A(t)
A = A0 + A1 * ca.sin(t)

# 定义对称矩阵 P 的变量
P = ca.SX.sym('P', 2, 2)
# 确保 P 是对称的
P_sym = ca.SX(2, 2)
P_sym[0, 0] = P[0, 0]
P_sym[0, 1] = P[0, 1]
P_sym[1, 0] = P[0, 1]  # 对称
P_sym[1, 1] = P[1, 1]

# 定义时间导数 P_dot 的变量
P_dot = ca.SX.sym('P_dot', 2, 2)
P_dot_sym = ca.SX(2, 2)
P_dot_sym[0, 0] = P_dot[0, 0]
P_dot_sym[0, 1] = P_dot[0, 1]
P_dot_sym[1, 0] = P_dot[0, 1]  # 对称
P_dot_sym[1, 1] = P_dot[1, 1]

# 构建 LMI 约束：A(t)^T * P + P * A(t) + P_dot < 0
LMI = ca.mtimes(A.T, P_sym) + ca.mtimes(P_sym, A) + P_dot_sym

# 定义优化问题
opt_variables = ca.vertcat(P[0, 0], P[0, 1], P[1, 1], P_dot[0, 0], P_dot[0, 1], P_dot[1, 1])  # 提取上三角部分变量
lbg = [-ca.inf] * LMI.shape[0] * LMI.shape[1]
ubg = [-1e-9] * LMI.shape[0] * LMI.shape[1]  # 使用一个小负数代替0以确保严格不等式

# 定义代价函数：trace(P)
cost_function = P[0, 0] + P[1, 1]

# 添加时间变量 t 作为参数
nlp = {'x': opt_variables, 'f': cost_function, 'g': ca.vec(LMI), 'p': t}

# 设置求解器并求解
solver = ca.nlpsol('solver', 'ipopt', nlp)

# 提供初始猜测值
initial_guess = np.ones(opt_variables.shape)

# 设置时间点
t_val = 0  # 可以设置为任何你感兴趣的时间点

# 求解
solution = solver(x0=initial_guess, lbg=lbg, ubg=ubg, p=t_val)

# 提取解并重构 P 和 P_dot
P_opt = np.zeros((2, 2))
P_dot_opt = np.zeros((2, 2))
solution_values = solution['x'].full().flatten()
P_opt[0, 0] = solution_values[0]
P_opt[0, 1] = solution_values[1]
P_opt[1, 0] = solution_values[1]  # 对称
P_opt[1, 1] = solution_values[2]
P_dot_opt[0, 0] = solution_values[3]
P_dot_opt[0, 1] = solution_values[4]
P_dot_opt[1, 0] = solution_values[4]  # 对称
P_dot_opt[1, 1] = solution_values[5]

print("Optimal P matrix:")
print(P_opt)
print("Optimal P_dot matrix:")
print(P_dot_opt)

# 检查 P 是否正定
eigvals = np.linalg.eigvals(P_opt)
print("Eigenvalues of P:", eigvals)
if np.all(eigvals > 0):
    print("P is positive definite.")
else:
    print("P is not positive definite.")
