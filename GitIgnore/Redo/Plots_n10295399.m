%% Create all the plots which were entered in the report. 
%The order of the figures are similar to the order in the report


r2d = [180/pi,1]; %Conversions from radians to degrees

%% Controlled Vs Uncontrolled (Linear System)

figure('Name','Controlled Vs Uncontrolled (Linear System)');
%Plot 1 Control Force 
subplot(3,2,1:2)
plot(Lin.t,Lin.F,'b',Lin_NC.t,Lin_NC.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Linear Controlled','F Linear Uncontrolled')
%Plot 2 x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'r',Lin_NC.t,r2d(1)*Lin_NC.x(:,1),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('x_1 Controlled','x_1 Uncontrolled','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'r',Lin_NC.t,r2d(1)*Lin_NC.x(:,2),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('x_2 Controlled','x_2 Uncontrolled','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'r',Lin_NC.t,r2d(2)*Lin_NC.x(:,3),'k--')
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('x_3 Controlled','x_3 Uncontrolled','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'r',Lin_NC.t,r2d(2)*Lin_NC.x(:,4),'k--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('x_4 Controlled','x_4 Uncontrolled','location','northeast')

%% Controlled Vs Uncontrolled (Nonlinear System)

figure('Name','Controlled Vs Uncontrolled (Nonlinear System)');

%Plot 1 Control Force 
subplot(3,2,1:2)
plot(NL.t,NL.F,'b',NL_NC.t,NL_NC.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Non-linear Controlled','F Non-linear Uncontrolled')

%Plot 2 x1
subplot(3,2,3)
plot(NL.t,r2d(1)*NL.x(:,1),'b',NL_NC.t,r2d(1)*NL_NC.x(:,1),'g--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('x_1 Controlled','x_1 Uncontrolled','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(NL.t,r2d(1)*NL.x(:,2),'b',NL_NC.t,r2d(1)*NL_NC.x(:,2),'g--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('x_2 Controlled','x_2 Uncontrolled','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(NL.t,r2d(2)*NL.x(:,3),'b',NL_NC.t,r2d(2)*NL_NC.x(:,3),'g--')
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('x_3 Controlled','x_3 Uncontrolled','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(NL.t,r2d(2)*NL.x(:,4),'b',NL_NC.t,r2d(2)*NL_NC.x(:,4),'g--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('x_4 Controlled','x_4 Uncontrolled','location','northeast')


%% OBSERVER ANDOUTPUT-FEEDBACKCONTROLLER

figure('Name','OBSERVER AND OUTPUT-FEEDBACKCONTROLLER');
%Plot 1 Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b',Lin.t,Lin.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Non-linear','F Linear')

%Plot 2 x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'r',Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',NL.t,r2d(1)*NL.x(:,1),'b',NL.t,r2d(1)*NL.x_hat(:,1),'g--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('x_1 Linear','x_1 hat Linear','x_1 Non Linear','x_1 hat Non Linear','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'r',Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',NL.t,r2d(1)*NL.x(:,2),'b',NL.t,r2d(1)*NL.x_hat(:,2),'g--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('x_2 Linear','x_2 hat Linear','x_2 Non Linear','x_2 hat Non Linear','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'r',Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',NL.t,r2d(2)*NL.x(:,3),'b',NL.t,r2d(2)*NL.x_hat(:,3),'g--')
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('x_3 Linear','x_3 hat Linear','x_3 Non Linear','x_3 hat Non Linear','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'r',Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',NL.t,r2d(2)*NL.x(:,4),'b',NL.t,r2d(2)*NL.x_hat(:,4),'g--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('x_4 Linear','x_4 hat Linear','x_4 Non Linear','x_4 hat Non Linear','location','northeast')


%% Starting position of -0.3m and -0.31m 



figure('Name','Starting position of -0.3m and -0.31m ');
%Plot 1 Control Force 
subplot(3,2,1:2)
plot(NL_neg3min.t,NL_neg3min.F,'r--', NL_neg3max.t,NL_neg3max.F,'b')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('-0.3m','-0.31')
%Plot 2 x1
subplot(3,2,3)
plot(NL_neg3min.t,r2d(1)*NL_neg3min.x(:,1),'r',NL_neg3max.t,r2d(1)*NL_neg3max.x(:,1),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('-0.3m','-0.31','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(NL_neg3min.t,r2d(1)*NL_neg3min.x(:,2),'r',NL_neg3max.t,r2d(1)*NL_neg3max.x(:,2),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('-0.3m','-0.31','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(NL_neg3min.t,r2d(2)*NL_neg3min.x(:,3),'r',NL_neg3max.t,r2d(2)*NL_neg3max.x(:,3),'k--')
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('-0.3m','-0.31','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(NL_neg3min.t,r2d(2)*NL_neg3min.x(:,4),'r',NL_neg3max.t,r2d(2)*NL_neg3max.x(:,4),'k--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('-0.3m','-0.31','location','northeast')


%% Starting position of 0.27m and 0.28m 



figure('Name','Starting position of 0.27m and 0.28m ');
%Plot 1 Control Force 
subplot(3,2,1:2)
plot(NL_pos3min.t,NL_pos3min.F,'r--', NL_pos3max.t,NL_pos3max.F,'b')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('0.27m','0.28m')
%Plot 2 x1
subplot(3,2,3)
plot(NL_pos3min.t,r2d(1)*NL_pos3min.x(:,1),'r',NL_pos3max.t,r2d(1)*NL_pos3max.x(:,1),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('0.27m','0.28m ','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(NL_pos3min.t,r2d(1)*NL_pos3min.x(:,2),'r',NL_pos3max.t,r2d(1)*NL_pos3max.x(:,2),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('0.27m','0.28m ','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(NL_pos3min.t,r2d(2)*NL_pos3min.x(:,3),'r',NL_pos3max.t,r2d(2)*NL_pos3max.x(:,3),'k--')
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('0.27m','0.28m ','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(NL_pos3min.t,r2d(2)*NL_pos3min.x(:,4),'r',NL_pos3max.t,r2d(2)*NL_pos3max.x(:,4),'k--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('0.27m','0.28m ','location','northeast')

%% Linear model with disturbance at 10 seconds



figure('Name','Linear model with disturbance at 10 seconds ');
%Plot 1 Control Force 
subplot(3,2,1:2)

plot(Lin_dis.t,Lin_dis.F,'b')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Linear')

%Plot 2 x1
subplot(3,2,3)
plot(Lin_dis.t,r2d(1)*Lin_dis.x(:,1),'r',Lin_dis.t,r2d(1)*Lin_dis.x_hat(:,1),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees')
title ('Angle of Rotating Actuator')
legend('x_1 Linear','x_1 hat Linear','location','northeast')


%Plot 3 x2
subplot(3,2,4)
plot(Lin_dis.t,r2d(1)*Lin_dis.x(:,2),'r',Lin_dis.t,r2d(1)*Lin_dis.x_hat(:,2),'k--')
grid on
xlabel('Seconds')
ylabel('Degrees Per Second')
title ('Velocity of Rotating Actuator')
legend('x_2 Linear','x_2 hat Linear','location','northeast')


%Plot 4 x3
subplot(3,2,5)
plot(Lin_dis.t,r2d(2)*Lin_dis.x(:,3),'r',Lin_dis.t,r2d(2)*Lin_dis.x_hat(:,3))
grid on
xlabel('Seconds')
ylabel('Meters')
title ('Position of Translational Oscillator')
legend('x_3 Linear','x_3 hat Linear','location','northeast')

%Plot 5 x4
subplot(3,2,6)
plot(Lin_dis.t,r2d(2)*Lin_dis.x(:,4),'r',Lin_dis.t,r2d(2)*Lin_dis.x_hat(:,4),'k--')
grid on
xlabel('Seconds')
ylabel('Meters per Second')
title ('Velocity of Translational Oscillator')
legend('x_4 Linear','x_4 hat Linear','location','northeast')