warning off
format compact
clc
clear vars
close all

colorOrder = get(gca,'colorOrder');
%% SETUP_SRV02_EXP14_2D_GANTRY
    %
    % Sets the necessary parameters to run the SRV02 Experiment #13: Position
    % Control of 2-DOF Robot laboratory using the "s_srv02_2d_robot" and
    % "q_srv02_2d_robot" Simulink diagrams.
    % 
    % Copyright (C) 2008 Quanser Consulting Inc.
    %
    clear vars;
    qc_get_step_size =1/1000; %0.001;
    deltaT = qc_get_step_size;
    Ts = qc_get_step_size;
    nPoints = (50 * 3000) -1;
%
%% Initialization Settings(DONT CHANGE ANYTHING !!)
    EXT_GEAR_CONFIG = 'HIGH';
    ENCODER_TYPE = 'E';
    TACH_OPTION = 'YES';
    LOAD_TYPE = 'NONE';
    K_AMP = 1;
    AMP_TYPE = 'VoltPAQ';
    VMAX_DAC = 10;
    ROTPEN_OPTION = '2DGANTRY-E';
    PEND_TYPE = 'MEDIUM_12IN';
    THETA_MAX = 35 * pi/180;
    ALPHA_MAX = 15.0 * pi/180;
    CONTROL_TYPE = 'AUTO';   
    X0 = pi/180*[0, 0, 0, 0];
    [ Rm, kt, km, Kg, eta_g, Beq, Jm, Jeq_noload, eta_m, K_POT, K_TACH, K_ENC, VMAX_AMP, IMAX_AMP ] = config_srv02( EXT_GEAR_CONFIG, ENCODER_TYPE, TACH_OPTION, AMP_TYPE, LOAD_TYPE );
    [ g, mp, Lp, lp, Jp_cm, Bp, RtpnOp, RtpnOff, K_POT_PEN ] = config_sp( PEND_TYPE, ROTPEN_OPTION );
    [ Lb, Jarm, K_POT_2DP, K_ENC_2DP ] = config_2d_gantry( Jeq_noload );
    K_ENC_2DIP = [-1,1].*K_ENC_2DP;
    wcf_1 = 2 * pi * 5;
    zetaf_1 = 0.9;
    wcf_2 = wcf_1;
    zetaf_2 = zetaf_1;

    %
%%%
%%%%
%%%%%
%%%%%% DO NOT CHANGE ANYTHING ABOVE THIS AREA !! Place your code below.
%%%%
%%%
%

%% Exercise 1 / 3.2

% Define the uncertain parameters Mp, Lp, Jp, and Co 
% using the command "ureal"

Mp = ureal('Mp', 0.1270, "Percentage", [-50, 50]);
Lp = ureal('Lp', 0.3111, "Percentage", [-50, 50]);
Jp = ureal('Jp', 0.0012, "Percentage", [-50, 50]);
Co = ureal('Co', 0.1285, "Percentage", [-10, 10]);

Lr = 0.1270; 
theta =  0;
alpha =  0;
dtheta =  0;
dalpha =  0;
Jr = 0.0083;
Dr = 0.0690;
g = 9.810;

Adelta = [0         0         1         0;
      0         0         0         1;
      0   ((Lr*Lp^3*Mp^2*dtheta^2)/2 + Lr*g*Lp^2*Mp^2)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) - ((dalpha*dtheta*Lp^4*Mp^2)/2 + 2*Jp*dalpha*dtheta*Lp^2*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)),- ((Jp*(4*Mp*alpha*dalpha*Lp^2 + 8*Dr))/2 + (Lp^4*Mp^2*alpha*dalpha)/2)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) - (- Lr*alpha*dtheta*Lp^3*Mp^2 + Dr*Lp^2*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)),-((alpha*dtheta*Lp^4*Mp^2)/2 + 2*Jp*alpha*dtheta*Lp^2*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr));
      0   (dtheta^2*(Lp^2*Lr^2*Mp^2 + Jr*Lp^2*Mp) + 2*Lp*Lr^2*Mp^2*g + 2*Jr*Lp*Mp*g)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) - (Lp^3*Lr*Mp^2*dalpha*dtheta)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)), (2*alpha*dtheta*(Lp^2*Lr^2*Mp^2 + Jr*Lp^2*Mp))/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) - (Lr*alpha*dalpha*Lp^3*Mp^2 + 2*Dr*Lr*Lp*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)),-(Lp^3*Lr*Mp^2*alpha*dtheta)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) ];
  
Bdelta = [0; 
      0;
      (4*Co*Jp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr)) + (Co*Lp^2*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr));
      (2*Co*Lp*Lr*Mp)/(Jr*Mp*Lp^2 + Jp*(4*Mp*Lr^2 + 4*Jr))];

  Cdelta = [1  0  0  0;
     0  1  0  0];
 
 Ddelta = [0;0];

 eig(Adelta.NominalValue)

%% Exercise 3 / 3.3
A = blkdiag(Adelta,Adelta);
B = blkdiag(Bdelta,Bdelta);
C = blkdiag(Cdelta,Cdelta);
D = blkdiag(Ddelta,Ddelta);

G_unc = uss(Adelta, Bdelta, Cdelta, Ddelta);
Pnom = ss(A.NominalValue, B.NominalValue, C, D); % only used by simulink
%% Exercise 4 / Question 3
% Design and compute the controller, and call it Chinf
% Chinf = hinfsyn(...)
%%%%% Uncertain models %%%%%
% First sample the uncertain model elements
[G_unc_samp, Samp_values] = usample(G_unc, 1000);

%% RUN FROM HERE %%
[~, Cov_info] = ucover(G_unc_samp, G_unc.NominalValue, 4, 'InputMult');

% Evaulate
%bodemag(Cov_info.W1)

%%%%% Weights %%%%%
Wim = 10*blkdiag(Cov_info.W1, Cov_info.W1);
Wim.InputName = "u";
Wim.OutputName = "yd";

wri = tf(1, [1, 1]);
Wr = blkdiag(wri, wri, wri, wri);
Wr.InputName = "r";
Wr.OutputName = "WrO";

wni = deg2rad(0.3); % magnitude
Wn = ss(blkdiag(wni, wni, wni, wni));
Wn.InputName = "n";
Wn.OutputName = "WnO";

Wu = ss(0.05*eye(2));
Wu.InputName = "u";
Wu.OutputName = "zu";

%%%%% Plant %%%%%
Gn = ss(A.NominalValue, B.NominalValue, C, D);
Gn.InputName = "GnI";
Gn.OutputName = "y";

%%%%% Connect %%%%%
sum1 = sumblk("GnI = ud + u", 2);      % to Gn
sum2 = sumblk("e = y - WrO", 4);       % to Wp
sum3 = sumblk("yt = y + WnO", 4);      % to K

%
% clf
% for k = 2:2
%     %%%%% Last weight here for the loop to work nicer %%%%%
%     zp = 0.99;
%     err = 2*[0.0025, 0.0251, 0.001];
%     wpf = [zp/err(1) zp/err(2), zp/err(3)];                   
%     wpi = tf([(7E-1+8E-3), 2],[8E-3, 1]);
%     Wp = wpf(k)*blkdiag(wpi, wpi, wpi, wpi);
%     Wp.InputName = "e";
%     Wp.OutputName = "zp";
%     dcgain(Wp)
%     
%     P = connect(Wim, Wr, Wp, Wn, Wu, Gn, sum1, sum2, sum3, ...
%         ["ud", "r", "n", "u"], ["yd", "zp", "zu", "yt"]);
%     
%     %%%%% Controller synthesis %%%%%
%     [K, N, gamma] = hinfsyn(P, 4, 2);
%     Chinf = hinfsyn(P, 4, 2);
%     
%     %%%%% Model uncertainty %%%%%
%     delta = 0.5;
%     deltaM = eye(2)*delta;
%     
%     %%%%% Closed loop %%%%%
%     PK = lft(P, K);
%     CL = lft(deltaM, PK);
% 
%     %%%%% Circular reference %%%%%
%     rt = 0:0.01:10;     % time
%     rr = 0.1;             % circle radius
%     rc = timeseries(rr*cos(rt), rt);    % cosine, for x
%     rs = timeseries(rr*sin(rt), rt);    % sine, for y
%     
%     % NB: To run the simulation, the nominal model has to be in the workspace with the variable name "Pnom"
%     %
%     %%%
%     %%%%
%     %%%%%
%     %%%%%% Closed loop simulation environment
%     %%%%
%     %%%
%     %
%     % NB: To run the simulation, the nominal model has to be loaded to the
%     % workspace with the variable name "Pnom".
%     
%     [ah,bh,ch,dh] = ssdata(Chinf);
%     
%     
%     %figure(1)
%     %clf;
%     simTime = 10;
%     xinit=(pi/180)*[3 3 0 0 3 3 0 0];
%     
%     try
%     sim('Simhinf.slx')
%     subplot(2,1,1)
%     hold on
%         plot(simStates.Time,simStates.Data(:,1),'--','linewidth',2)
%         plot(simStates.Time,simStates.Data(:,3),':', 'linewidth',2)    
%         %plot(simStates.Time,simStates.Data(:,2),'-.','linewidth',2)
%         %plot(simStates.Time,simStates.Data(:,4),'--','linewidth',2)
%         %legend('thetaX','alphaX','thetaY','alphaY','NthetaX','NalphaX','NthetaY','NalphaY')
%         ylabel('Angle [DEG]')
%         xlim([0, 10])
%     
%     subplot(2,1,2)
%     hold on
%         plot(simVoltage.Time, simVoltage.Data(:,1),'--','linewidth',2)
%         %plot(simVoltage.Time, simVoltage.Data(:,2),'--','linewidth' ,2)
%         %legend('Voltage X','Voltage Y')
%         ylabel('Voltage [V]')
%         axis([0 10 -12 15])
%      catch e
%         disp('Simulation failed')
%     end
%     OCL=1;
% end
% subplot(2,1,1)
% legend('thetaX1','alphaX1','thetaX2','alphaX2','thetaX3','alphaX3')
% subplot(2,1,2)
% legend('Voltage X1','Voltage X2', 'Voltage X3')
% 
% save("Chinf.mat", "Chinf")
%
%load('labb_LQR_controller.mat')
%open('ExperimentRobustControl')
% For the lab
k = 1;
zp = 0.99;
err = 2*[0.0025, 0.0251, 0.001];
wpf = [zp/err(1) zp/err(2), zp/err(3)];                   
% wpi = tf([(7E-1+8E-3), 2],[8E-3, 1]);
wpi = tf([1/7, 1],[8E+3, 1]+1);
Wp = wpf(k)*blkdiag(wpi, wpi, wpi, wpi);
Wp.InputName = "e";
Wp.OutputName = "zp";
dcgain(Wp)

P = connect(Wim, Wr, Wp, Wn, Wu, Gn, sum1, sum2, sum3, ...
    ["ud", "r", "n", "u"], ["yd", "zp", "zu", "yt"]);

%%%%% Controller synthesis %%%%%
[K, N, gamma] = hinfsyn(P, 4, 2);
Chinf = hinfsyn(P, 4, 2);

[ah,bh,ch,dh] = ssdata(Chinf);

