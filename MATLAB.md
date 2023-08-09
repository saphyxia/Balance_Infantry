# MATLAB

### 创建动态变量

```matlab
syms t x(t) phi(t) x_m(t)

x = symfun(x(t),t)
dx = diff(x)
ddx = diff(dx)

phi = symfun(phi(t),t)
dphi = diff(phi)
ddphi = diff(dphi)

x_m = symfun(x_m(t),t)
dx_m = diff(x_m)
ddx_m = diff(dx_m)

```

### 创建模型变量

```matlab

syms L T_0 T_1 T_2 T T_m m_b m_m m_w l z_m g i_b i_m

% 计算系统动能
T_0 = 1/2*m_w*dx^2
T_1 = 1/2*m_b*[diff((x+l*sin(phi)),t)^2 + diff(l*cos(phi),t)^2]
T_1= simplify(T_1)
T_2 = 1/2*m_m*[diff((x+(l+z_m)*sin(phi)+x_m*cos(phi)),t)^2+diff(((l+z_m)*cos(phi)+x_m*sin(phi)),t)^2]
T_2= simplify(T_2)

% 计算系统势能
V_1 = m_b*g*l*cos(phi)
V_2 = m_m*g*(l+z_m)*cos(phi)

% 拉格朗日函数
L = T_0+T_1+T_2-V_1-V_2

% 通过对x,dx,x_m,dx_m,phi和dphi求导得到拉格朗日方程
LE1 = diff(diff(L,dx),t)-diff(L,x)

LE2 = diff(diff(L,dphi),t)-diff(L,phi)

LE3 = diff(diff(L,dx_m),t)-diff(L,x_m)

% 求解状态向量
% 注：此处开始报错，solve函数无法求关于符号函数的解
% 尝试解决方法：将上文中的动态变量转换为符号变量，但最终求解出错误的状态空间方程，此demo后续通过python得以完成
solve_b = solve([LE1==T,LE2==0,LE3==T_m],[ddx,ddphi,ddx_m])

solve_x = [dx,dphi,dx_m,solve_b.ddx,solve_b.ddphi,solve_b.ddx_m]

f_dot_x = jacobian(solve_x,[x,phi,x_m,dx,dphi,dx_m])

f_dot_u = jacobian(solve_x,[T,T_m])

A = subs(f_dot_x,[x,phi,x_m,dx,dphi,dx_m], [0,0,0,0,0,0])

B = subs(f_dot_u,[x,phi,x_m,dx,dphi,dx_m], [0,0,0,0,0,0])


```

