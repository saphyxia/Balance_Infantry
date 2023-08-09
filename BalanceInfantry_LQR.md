# BalanceInfantry LQR

## MATLAB

```matlab
% 整车质量
M=18.55
% 驱动轮质量
m_w=0.65
% 驱动轮轮距
wheelbase=0.51
% 驱动轮半径
wheelradius=0.105
% 机器人除轮子和动量块外的质量
m=M - 2*m_w
% 是单个动量块的质量
m_b= 0.5
% 驱动轮绕电机旋转轴的转动惯量
i_w = 0.5*m_w*wheelradius^2
% 轮心与机器人质心距离的垂直分量
l=0.1
% base_link坐标系中动量块坐标的y轴分量
y_b=0.08
% 动量块与机器人质心距离的垂直分量
z_b=0.05
% 机器人绕 base_link 坐标 y 轴的转动惯量
i_m=1/3*m*l^2
% 重力加速度
g=9.8

% matrix A
    a_5_2 = -(wheelradius^2 * g * (l^2 * m^2 + 2 * m_b * l^2 * m + 2 * i_m * m_b)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
                  
   a_5_3 = -(wheelradius^2 * g * l * m * m_b) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   a_5_4 = a_5_3
   
   a_7_2 =(g * l * m *(2 * i_w + wheelradius^2 * m + 2 * wheelradius^2 * m_b + 2 * wheelradius^2 * m_w)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   a_7_3 = (g * m_b * (2 * i_w + wheelradius^2 * m + 2 * wheelradius^2 * m_w)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   a_7_4 = a_7_3
   
   a_8_2 =(g * (i_m - l * m * z_b) *(2 * i_w + wheelradius^2 * m + 2 * wheelradius^2 * m_b + 2 * wheelradius^2 * m_w)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   a_8_3 = -(g * m_b * (2 * i_w * l + 2 * i_w * z_b + 2 * wheelradius^2 * l * m_w +wheelradius^2 * m * z_b + 2 * wheelradius^2 * m_w * z_b)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   a_8_4 = a_8_3
   
   a_9_2 = a_8_2
   
   a_9_3 = a_8_3
   
   a_9_4 = a_8_4
   
A=[
0  0  0  0  0  1.0 0   0    0   0 ; 
0  0  0  0  0  0   1.0 0    0   0 ;
0  0  0  0  0  0   0   1.0  0   0 ;
0  0  0  0  0  0   0   0    1.0 0 ;
0  0  0  0  0  0   0   0    0   1.0 ; 
0  0  a_5_2  a_5_3  a_5_4  0  0  0  0  0 ;
0  0  0  0  0  0  0  0  0  0 ; 
0  0  a_7_2  a_7_3  a_7_4  0  0  0  0  0 ;
0  0  a_8_2  a_8_3  a_8_4  0  0  0  0  0 ; 
0  0  a_9_2  a_9_3  a_9_4  0  0  0  0  0 ]
   
% matrix B
   b_5_0 = (wheelradius * (m * l^2 + wheelradius * m * l + i_m)) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
    
   b_5_1 = b_5_0
   
   b_5_2 = (wheelradius^2 * l * m * z_b) /(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
                  
   b_5_3 = b_5_2
   
   b_6_0 = -wheelradius / (wheelbase * (4 * m_w * wheelradius^2 + i_w))
   
   b_6_1 = -b_6_0
   
   b_6_2 = (2 * wheelradius^2 * y_b) / (wheelbase^2 * (4 * m_w * wheelradius^2 + i_w))
   
   b_6_3 = -b_6_2
   
   b_7_0 = -(2 * i_w + wheelradius^2 * m + 2 * wheelradius^2 * m_w + wheelradius * l * m) / (2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
   
   b_7_1 = b_7_0
   
   b_7_2 = -(z_b * (2 * i_w + wheelradius^2 * m + 2 * wheelradius^2 * m_w)) / (2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w)
                  
   b_7_3 = b_7_2
   
   b_9_0 =(2 * i_w^2 * l * wheelbase + 2 * i_w^2 * wheelbase * z_b - 4 * wheelradius^3 * i_m * m_w * wheelbase + wheelradius^3 * i_m * m * y_b + 2 * wheelradius^3 * i_m * m_w * y_b + 8 * wheelradius^4 * l * m_w^2 * wheelbase +8 * wheelradius^4 * m_w^2 * wheelbase * z_b - wheelradius * i_m * i_w * wheelbase + 2 * wheelradius * i_m * i_w * y_b + 2 * wheelradius^3 * l^2 * m * m_w * y_b + 10 * wheelradius^2 * i_w * l * m_w * wheelbase + 2 * wheelradius * i_w * l^2 * m * y_b + wheelradius^2 * i_w * m * wheelbase * z_b + 10 * wheelradius^2 * i_w * m_w * wheelbase * z_b + 4 * wheelradius^4 * m * m_w * wheelbase * z_b + 4 * wheelradius^3 * l * m * m_w * wheelbase * z_b + wheelradius * i_w * l * m * wheelbase * z_b) / (wheelbase * (4 * m_w * wheelradius^2 + i_w) *  (2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w))
   
   b_9_1 =(2 * i_w^2 * l * wheelbase + 2 * i_w^2 * wheelbase * z_b -4 * wheelradius^3 * i_m * m_w * wheelbase - wheelradius^3 * i_m * m * y_b -2 * wheelradius^3 * i_m * m_w * y_b + 8 * wheelradius^4 * l * m_w^2 * wheelbase +8 * wheelradius^4 * m_w^2 * wheelbase * z_b - wheelradius * i_m * i_w * wheelbase -2 * wheelradius * i_m * i_w * y_b - 2 * wheelradius^3 * l^2 * m * m_w * y_b +10 * wheelradius^2 * i_w * l * m_w * wheelbase - 2 * wheelradius * i_w * l^2 * m * y_b +wheelradius^2 * i_w * m * wheelbase * z_b + 10 * wheelradius^2 * i_w * m_w * wheelbase * z_b +4 * wheelradius^4 * m * m_w * wheelbase * z_b +4 * wheelradius^3 * l * m * m_w * wheelbase * z_b + wheelradius * i_w * l * m * wheelbase * z_b) /(wheelbase * (4 * m_w * wheelradius^2 + i_w) *(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w))
        
   b_9_2 =(-4 * m * wheelradius^4 * l^2 * m_w * y_b^2 +8 * wheelradius^4 * l * m_w^2 * wheelbase^2 * z_b +8 * wheelradius^4 * m_w^2 * wheelbase^2 * z_b^2 +4 * m * wheelradius^4 * m_w * wheelbase^2 * z_b^2 -4 * i_m * wheelradius^4 * m_w * y_b^2 - 2 * i_m * m * wheelradius^4 * y_b^2 -4 * m * wheelradius^2 * i_w * l^2 * y_b^2 +10 * wheelradius^2 * i_w * l * m_w * wheelbase^2 * z_b +10 * wheelradius^2 * i_w * m_w * wheelbase^2 * z_b^2 +m * wheelradius^2 * i_w * wheelbase^2 * z_b^2 -4 * i_m * wheelradius^2 * i_w * y_b^2 + 2 * i_w^2 * l * wheelbase^2 * z_b +2 * i_w^2 * wheelbase^2 * z_b^2) /(wheelbase^2 * (4 * m_w * wheelradius^2 + i_w) *(2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m +2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w))
        
   b_9_3 =(8 * m * wheelradius^4 * l^2 * m_w^2 * wheelbase^2 +4 * m * m_b * wheelradius^4 * l^2 * m_w * y_b^2 +8 * m_b * wheelradius^4 * l * m_w^2 * wheelbase^2 * z_b +8 * m_b * wheelradius^4 * m_w^2 * wheelbase^2 * z_b^2 +8 * i_m * wheelradius^4 * m_w^2 * wheelbase^2 +4 * m * m_b * wheelradius^4 * m_w * wheelbase^2 * z_b^2 +4 * i_m * m * wheelradius^4 * m_w * wheelbase^2 +4 * i_m * m_b * wheelradius^4 * m_w * y_b^2 +2 * i_m * m * m_b * wheelradius^4 * y_b^2 +10 * m * wheelradius^2 * i_w * l^2 * m_w * wheelbase^2 +4 * m * m_b * wheelradius^2 * i_w * l^2 * y_b^2 +10 * m_b * wheelradius^2 * i_w * l * m_w * wheelbase^2 * z_b + 10 * m_b * wheelradius^2 * i_w * m_w * wheelbase^2 * z_b^2 + 10 * i_m * wheelradius^2 * i_w * m_w * wheelbase^2 + m * m_b * wheelradius^2 * i_w * wheelbase^2 * z_b^2 +  i_m * m * wheelradius^2 * i_w * wheelbase^2 +  4 * i_m * m_b * wheelradius^2 * i_w * y_b^2 +  2 * m * i_w^2 * l^2 * wheelbase^2 + 2 * m_b * i_w^2 * l * wheelbase^2 * z_b + 2 * m_b * i_w^2 * wheelbase^2 * z_b^2 + 2 * i_m * i_w^2 * wheelbase^2) / (m_b * wheelbase^2 * (4 * m_w * wheelradius^2 + i_w) * (2 * i_m * i_w + 2 * i_w * l^2 * m + wheelradius^2 * i_m * m + 2 * wheelradius^2 * i_m * m_w + 2 * wheelradius^2 * l^2 * m * m_w))
        
   b_8_0 = b_9_1
   
   b_8_1 = b_9_0
   
   b_8_2 = b_9_3
   
   b_8_3 = b_9_2

B=[
0  0  0  0 ;
0  0  0  0 ;
0  0  0  0 ;
0  0  0  0 ;
0  0  0  0 ;
b_5_0 b_5_1 b_5_2 b_5_3;
b_6_0 b_6_1 b_6_2 b_6_3;
b_7_0 b_7_1 b_7_2 b_7_3;
b_8_0 b_8_1 b_8_2 b_8_3;
b_9_0 b_9_1 b_9_2 b_9_3]

% matrix Q
Q=[
20  0  0  0  0  0  0  0  0  0 ;
0 1500  0  0  0  0  0  0  0  0 ;
0  0  5000  0  0  0  0  0  0  0 ;
0  0  0  2000 0  0  0  0  0  0 ;
0  0  0  0  2000 0  0  0  0  0 ;
0  0  0  0  0 100  0  0  0  0 ;
0  0  0  0  0  0  1  0  0  0 ;
0  0  0  0  0  0  0  250  0  0 ;
0  0  0  0  0  0  0  0  1  0 ; 
0  0  0  0  0  0  0  0  0  1 ;]

% matrix R
R=[
0.5  0  0  0  ;
0  0.5  0  0  ;
0  0  1  0  ;
0  0  0  1 ;]

% matrix K
k=-lqr(A,B,Q,R)
```