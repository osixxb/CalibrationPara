% data = load('NO202102920210522_bd2.txt');% check1.txt为补前，check2.txt为补后

% data = load('check210506_23.23.08.txt');
% data = load('check210530_22.33.25.txt');
data = load('check220021_11.33.46.txt');
[n, m] = size(data);
phiE = data(:,1); % 横摇角误差,单位arcmin
phiN = data(:,2); % 纵摇角误差,单位arcmin
phiU = data(:,3); % 航向角误差,单位arcmin
dvE = data(:,4); % 东速误差,单位m/s
dvN = data(:,5); % 北速误差,单位m/s
dvU = data(:,6); % 垂速误差,单位m/s
BGx = data(:,7); % x陀螺常值漂移,单位deg/h
BGy = data(:,8); % y陀螺常值漂移,单位deg/h
BGz = data(:,9); % z陀螺常值漂移,单位deg/h
BAx = data(:,10); % x加速度计零偏,单位mg
BAy = data(:,11); % y加速度计零偏,单位mg
BAz = data(:,12); % z加速度计零偏,单位mg
dKGx = data(:,13); % x陀螺标度误差,单位ppm
dKGy = data(:,14); % y陀螺标度误差,单位ppm
dKGz = data(:,15); % z陀螺标度误差,单位ppm
dUGxy = data(:,16); % y对z陀螺安装误差,单位arcmin
dUGxz = data(:,17); % z对y陀螺安装误差,单位arcmin
dUGyz = data(:,18); % z对x陀螺安装误差,单位arcmin
dKAx = data(:,19); % x陀螺标度误差,单位ppm
dKAy = data(:,20); % y陀螺标度误差,单位ppm
dKAz = data(:,21); % z陀螺标度误差,单位ppm
dUAxy = data(:,22); % x对z加速度计安装误差,单位arcmin
dUAxz = data(:,23); % x对y加速度计安装误差,单位arcmin
dUAyx = data(:,25); % y对x加速度计安装误差,单位arcmin
dUAyz = data(:,24); % y对z加速度计安装误差,单位arcmin
dUAzx = data(:,26); % z对y加速度计安装误差,单位arcmin
dUAzy = data(:,27); % z对x加速度计安装误差,单位arcmin
pphiE = data(:,28); % 横摇角误差,单位arcmin
pphiN = data(:,29); % 纵摇角误差,单位arcmin
pphiU = data(:,30); % 航向角误差,单位arcmin
pdvE = data(:,31); % 东速误差,单位m/s
pdvN = data(:,32); % 北速误差,单位m/s
pdvU = data(:,33); % 垂速误差,单位m/s
pBGx = data(:,34); % x陀螺常值漂移,单位deg/h
pBGy = data(:,35); % y陀螺常值漂移,单位deg/h
pBGz = data(:,36); % z陀螺常值漂移,单位deg/h
pBAx = data(:,37); % x加速度计零偏,单位mg
pBAy = data(:,38); % y加速度计零偏,单位mg
pBAz = data(:,39); % z加速度计零偏,单位mg
pdKGx = data(:,40); % x陀螺标度误差,单位ppm
pdKGy = data(:,41); % y陀螺标度误差,单位ppm
pdKGz = data(:,42); % z陀螺标度误差,单位ppm
pdUGxy = data(:,43); % y对z陀螺安装误差,单位arcmin
pdUGxz = data(:,44); % z对y陀螺安装误差,单位arcmin
pdUGyz = data(:,45); % z对x陀螺安装误差,单位arcmin
pdKAx = data(:,46); % x陀螺标度误差,单位ppm
pdKAy = data(:,47); % y陀螺标度误差,单位ppm
pdKAz = data(:,48); % z陀螺标度误差,单位ppm
pdUAxy = data(:,49); % x对z加速度计安装误差,单位arcmin
pdUAxz = data(:,50); % x对y加速度计安装误差,单位arcmin
pdUAyx = data(:,52); % y对x加速度计安装误差,单位arcmin
pdUAyz = data(:,51); % y对z加速度计安装误差,单位arcmin
pdUAzx = data(:,53); % z对y加速度计安装误差,单位arcmin
pdUAzy = data(:,54); % z对x加速度计安装误差,单位arcmin

time = (1:n)/3600;
% close all;
%%
figure;
subplot(2,2,1);
plot(time,phiE,time,phiN,time,phiU);
legend('phiE','phiN','phiU');
ylabel('姿态角误差(arcmin)');
subplot(2,2,2);
plot(time,pphiE,time,pphiN,time,pphiU);
legend('pphiE','pphiN','pphiU');
ylabel('姿态角误差(arcmin)');
subplot(2,2,3);
plot(time,dvE,time,dvN,time,dvU);
legend('dvE','dvN','dvU');
ylabel('速度误差(m/s)');
xlabel('时间(h)');
subplot(2,2,4);
plot(time,pdvE,time,pdvN,time,pdvU);
legend('pdvE','pdvN','pdvU');
ylabel('速度误差(m/s)');
xlabel('时间(h)');
%%
figure;
subplot(3,2,1);
plot(time,BGx,time,BGy,time,BGz);
legend('BGx','BGy','BGz');
ylabel('陀螺常值漂移(deg/h)');
% xlabel('时间(s)');
subplot(3,2,3);
plot(time,dKGx,time,dKGy,time,dKGz);
legend('dKGx','dKGy','dKGz');
ylabel('陀螺标度误差(ppm)');
% xlabel('时间(s)');
subplot(3,2,5);
plot(time,dUGxy,time,dUGxz,time,dUGyz);
legend('dUGxy','dUGxz','dUGyz');
ylabel('陀螺安装误差(arcmin)');
xlabel('时间(h)');
subplot(3,2,2);
plot(time,BAx,time,BAy,time,BAz);
legend('BAx','BAy','BAz');
ylabel('加速度计零偏(mg)');
% xlabel('时间(s)');
subplot(3,2,4);
plot(time,dKAx,time,dKAy,time,dKAz);
legend('dKAx','dKAy','dKAz');
ylabel('加速度计标度误差(ppm)');
% xlabel('时间(s)');
subplot(3,2,6);
plot(time,dUAxy,time,dUAxz,time,dUAyx,time,dUAyz,time,dUAzx,time,dUAzy);
legend('dUAxy','dUAxz','dUAyx','dUAyz','dUAzx','dUAzy');
ylabel('加速度计安装误差(arcmin)');
xlabel('时间(h)');

%%
figure;
subplot(3,2,1);
plot(time,pBGx,time,pBGy,time,pBGz);
legend('pBGx','pBGy','pBGz');
ylabel('陀螺常值漂移(deg/h)');
% xlabel('时间(s)');
subplot(3,2,2);
plot(time,pBAx,time,pBAy,time,pBAz);
legend('pBAx','pBAy','pBAz');
ylabel('加速度计零偏(mg)');
% xlabel('时间(s)');
subplot(3,2,3);
plot(time,pdKGx,time,pdKGy,time,pdKGz);
legend('pdKGx','pdKGy','pdKGz');
ylabel('陀螺标度误差(ppm)');
% xlabel('时间(s)');
subplot(3,2,5);
plot(time,pdUGxy,time,pdUGxz,time,pdUGyz);
legend('pdUGxy','pdUGxz','pdUGyz');
ylabel('陀螺安装误差(arcmin)');
% xlabel('时间(s)');
subplot(3,2,4);
plot(time,pdKAx,time,pdKAy,time,pdKAz);
legend('pdKAx','pdKAy','pdKAz');
ylabel('加速度计标度误差(ppm)');
% xlabel('时间(s)');
subplot(3,2,6);
plot(time,pdUAxy,time,pdUAxz,time,pdUAyx,time,pdUAyz,time,pdUAzx,time,pdUAzy);
legend('pdUAxy','pdUAxz','pdUAyx','pdUAyz','pdUAzx','pdUAzy');
ylabel('加速度计安装误差(arcmin)');
xlabel('时间(h)');
