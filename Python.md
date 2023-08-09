# Python

````python
# 导入sympy库
import sympy as smp
from sympy.physics.mechanics import dynamicsymbols

# 创建符号变量
m_w,i_c,m_m,m_b,l_m,l,g,t = smp.symbols('m_w,i_c,m_m,m_b,l_m,l,g,t')

# 创建动态变量
x,phi,y_m = dynamicsymbols('x,phi,y_m')
xd,phid,y_md = dynamicsymbols('x,phi,y_m',1)
xdd,phidd,y_mdd = dynamicsymbols('x,phi,y_m',2)

# 驱动轮动能
T_w = 1/2 * m_w * xd**2 

# 机体动能
x_body = x + l * smp.sin(phi)
z_body = l * smp.cos(phi)
T_body = 1/2 * m_b * (smp.diff(x_body, t)**2 + smp.diff(z_body, t)**2)
T_body = smp.simplify(T_body)

# 动量块动能
x_block = x + y_m * smp.cos(phi) + l_m * smp.sin(phi)
z_block = l_m * smp.cos(phi) - y_m * smp.sin(phi)
T_x = 1/2 * m_m * smp.diff(x_block, t)**2
T_z = 1/2 * m_m * smp.diff(z_block, t)**2
T_block = T_x + T_z
T_block = smp.simplify(T_block)

# 机体、动量块势能
V_body = m_b * g * z_body
V_block = m_m * g * z_block 

# 拉格朗日量
T = T_w + T_body + T_block
V = V_body + V_block
L = T - V

# 拉格朗日函数
LE1 = smp.diff(smp.diff(L,xd),t).simplify() - smp.diff(L, x)
LE3 = smp.diff(smp.diff(L,phid),t).simplify() - smp.diff(L, phi)
LE4 = smp.diff(smp.diff(L,y_md),t).simplify() - smp.diff(L, y_m)

T,T_m = smp.symbols('T,T_m')

# 求解状态向量
LE = smp.Matrix([LE1-T,LE3,LE4-T_m])
solve_value = smp.solve(LE,[xdd,phidd,y_mdd])
x_dot = smp.Matrix([xd,phid,y_md,solve_value[xdd],solve_value[phidd],solve_value[y_mdd]])


f_dot_x = smp.Matrix([[x_dot.diff(x),x_dot.diff(phi),x_dot.diff(y_m),x_dot.diff(xd),x_dot.diff(phid),x_dot.diff(y_md)]])

f_dot_u = smp.Matrix([[x_dot.diff(T),x_dot.diff(T_m)]])

A = smp.simplify(f_dot_x.evalf(subs={x:0,phi:0,y_m:0,xd:0,phid:0,y_md:0}))

B = smp.simplify(f_dot_u.evalf(subs={x:0,phi:0,y_m0,xd:0,phid:0,y_md:0}))

print(A)
print

print(B)
print

````
