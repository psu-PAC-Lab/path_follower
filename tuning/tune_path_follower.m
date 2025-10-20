s = tf('s');

%% path-normal controller tuning

% cross-track error dynamics: d/dt(y_r - y) = v*sin(dth)
% y_r / y are the normal deviations from the planned path (s.t. y_r = 0)
% v is velocity and dth = th-th_ff where th is the heading
% control law: th_cmd = atan((1/v)*kn*(y_r-y)) + th_ff
% turn rate (om) dynamics are first order with time constant tau_om

% PI controller for th:
tau_om = 0.3;
P_om = 1/(tau_om*s + 1);
P_th = (1/s)*P_om;

%controlSystemDesigner(P_th, 1);
kth = 1.5;
C_th = kth;

% closed loop heading dynamics
L_th = C_th*P_th;
CL_th = L_th/(1+L_th);
CL_th = minreal(CL_th, 1e-3);

% for cross-track error controller, in small angle limit, v's cancel out by 
% design such that the plant transfer function is:
P_1 = CL_th/s;

% call control system designer
%controlSystemDesigner(P_1, 1);

% this gain achieves approximate critical damping
kn = 0.6;
C_n = kn;


%% path tangential controller tuning

% tangential error dynamics: d/dt(x_r - x) = v*cos(th)
% velocity dynamics are first order with time constant tau_v
tau_v = 0.2;
P_v = 1/(tau_v*s+1);
P_x = (1/s)*P_v;

% call control system designer
%controlSystemDesigner(P_x, 1);

% this gain achieves critical damping
kt = 1.9;


