function [LTI, xlb, xub, ulb, uub] = Suspension_Model()
m_b = 690;
m_w1 = 40.5;
m_w2 = 45.4;
k_s1 = 17e3;
k_s2 = 22e3;
c_s1 = 1500;
c_s2 = c_s1;
a = 1.25;
b = 1.51;
k_u1 = 192e3;
k_u2 = k_u1;
I_yy = 1222;

A = [
    -(c_s1+c_s2)/m_b, (a*c_s1-b*c_s2)/m_b, c_s1/m_b, c_s2/m_b, -k_s1/m_b, -k_s2/m_b, 0, 0;
    (a*c_s1-b*c_s2)/I_yy, -(a^2*c_s1+b^2*c_s2)/I_yy, -a*c_s1/I_yy, b*c_s2/I_yy, a*k_s1/I_yy, -b*k_s1/I_yy, 0, 0;
    c_s1/m_w1, -a*c_s1/m_w1, -c_s1/m_w1, 0, k_s1/m_w1, 0, -k_u1/m_w1, 0;
    c_s2/m_w2, b*c_s2/m_w2, 0, -c_s2/m_w2, 0, k_s2/m_w2, 0, -k_u2/m_w2;
    1, -a, -1, 0, 0, 0, 0, 0;
    1, b, 0, -1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0, 0, 0;
    0, 0, 0, 1, 0, 0, 0, 0
];

B=[
    1/m_b, 1/m_b, 0, 0, 0;
    -a/I_yy, b/I_yy, 0, 0, 1/I_yy;
    -1/m_w1, 0, 0, 0, 0;
    0, -1/m_w2, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, -1, 0, 0;
    0, 0, 0, -1, 0
];

C = [
    -(c_s1+c_s2)/m_b, (a*c_s1-b*c_s2)/m_b, c_s1/m_b, c_s2/m_b, -k_s1/m_b, -k_s2/m_b, 0, 0;
    (a*c_s1-b*c_s2)/I_yy, -(a^2*c_s1+b^2*c_s2)/I_yy, -a*c_s1/I_yy, b*c_s2/I_yy, a*k_s1/I_yy, -b*k_s1/I_yy, 0, 0;
    0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 1, 0, 0;
];

D = [
    1/m_b, 1/m_b, 0, 0, 0;
    -a/I_yy, b/I_yy, 0, 0, 1/I_yy;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
];
%%
P = ss(A,B,C,D);
ts=0.01;          % sampling time
P_dis=c2d(P,ts);  % discrete system P
LTI.A=P_dis.A;
LTI.B=P_dis.B;
LTI.C=P_dis.C;
LTI.D=P_dis.D;

xlb = [-inf(), -inf(), -inf(), -inf(), -0.10, -0.10, -inf(), -inf()]';
xub = [inf(), inf(), inf(), inf(), 0.08, 0.08, (m_b*b/(a+b)+m_w1)*9.81/k_s1, (m_b*a/(a+b)+m_w2)*9.81/k_s2]';
ulb = [-3500, -3500, -inf(), -inf(), -inf()]';
uub = [3500, 3500, inf(), inf(), inf()]';
% 
% T = 5;
% step = 0:ts:T;
% x0 = [0;0;0;0;0.02;-0.02;0;0];
% % Force_F, Force_R, M, WheelSpd_F, WheelSpd_R
% u0 = [0;0;0;0;0];
% x = x0;
% 
% for i=1:length(step)-1
%     x_new = LTI.A*x(:,end)+LTI.B*u0;
%     x=[x, x_new];
% end

% figure()
% stairs(step, x(6,:)) % 用阶梯图画出状态保持器的效果
% xlabel('Time [s]')
% ylabel('State x6')
% title('Zero-Order Hold Effect')
% grid on
end