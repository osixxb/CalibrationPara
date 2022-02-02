% data = load('NO202102920210522_bd2.txt');% check1.txtΪ��ǰ��check2.txtΪ����

% data = load('check210506_23.23.08.txt');
% data = load('check210530_22.33.25.txt');
data = load('check220021_11.33.46.txt');
[n, m] = size(data);
phiE = data(:,1); % ��ҡ�����,��λarcmin
phiN = data(:,2); % ��ҡ�����,��λarcmin
phiU = data(:,3); % ��������,��λarcmin
dvE = data(:,4); % �������,��λm/s
dvN = data(:,5); % �������,��λm/s
dvU = data(:,6); % �������,��λm/s
BGx = data(:,7); % x���ݳ�ֵƯ��,��λdeg/h
BGy = data(:,8); % y���ݳ�ֵƯ��,��λdeg/h
BGz = data(:,9); % z���ݳ�ֵƯ��,��λdeg/h
BAx = data(:,10); % x���ٶȼ���ƫ,��λmg
BAy = data(:,11); % y���ٶȼ���ƫ,��λmg
BAz = data(:,12); % z���ٶȼ���ƫ,��λmg
dKGx = data(:,13); % x���ݱ�����,��λppm
dKGy = data(:,14); % y���ݱ�����,��λppm
dKGz = data(:,15); % z���ݱ�����,��λppm
dUGxy = data(:,16); % y��z���ݰ�װ���,��λarcmin
dUGxz = data(:,17); % z��y���ݰ�װ���,��λarcmin
dUGyz = data(:,18); % z��x���ݰ�װ���,��λarcmin
dKAx = data(:,19); % x���ݱ�����,��λppm
dKAy = data(:,20); % y���ݱ�����,��λppm
dKAz = data(:,21); % z���ݱ�����,��λppm
dUAxy = data(:,22); % x��z���ٶȼư�װ���,��λarcmin
dUAxz = data(:,23); % x��y���ٶȼư�װ���,��λarcmin
dUAyx = data(:,25); % y��x���ٶȼư�װ���,��λarcmin
dUAyz = data(:,24); % y��z���ٶȼư�װ���,��λarcmin
dUAzx = data(:,26); % z��y���ٶȼư�װ���,��λarcmin
dUAzy = data(:,27); % z��x���ٶȼư�װ���,��λarcmin
pphiE = data(:,28); % ��ҡ�����,��λarcmin
pphiN = data(:,29); % ��ҡ�����,��λarcmin
pphiU = data(:,30); % ��������,��λarcmin
pdvE = data(:,31); % �������,��λm/s
pdvN = data(:,32); % �������,��λm/s
pdvU = data(:,33); % �������,��λm/s
pBGx = data(:,34); % x���ݳ�ֵƯ��,��λdeg/h
pBGy = data(:,35); % y���ݳ�ֵƯ��,��λdeg/h
pBGz = data(:,36); % z���ݳ�ֵƯ��,��λdeg/h
pBAx = data(:,37); % x���ٶȼ���ƫ,��λmg
pBAy = data(:,38); % y���ٶȼ���ƫ,��λmg
pBAz = data(:,39); % z���ٶȼ���ƫ,��λmg
pdKGx = data(:,40); % x���ݱ�����,��λppm
pdKGy = data(:,41); % y���ݱ�����,��λppm
pdKGz = data(:,42); % z���ݱ�����,��λppm
pdUGxy = data(:,43); % y��z���ݰ�װ���,��λarcmin
pdUGxz = data(:,44); % z��y���ݰ�װ���,��λarcmin
pdUGyz = data(:,45); % z��x���ݰ�װ���,��λarcmin
pdKAx = data(:,46); % x���ݱ�����,��λppm
pdKAy = data(:,47); % y���ݱ�����,��λppm
pdKAz = data(:,48); % z���ݱ�����,��λppm
pdUAxy = data(:,49); % x��z���ٶȼư�װ���,��λarcmin
pdUAxz = data(:,50); % x��y���ٶȼư�װ���,��λarcmin
pdUAyx = data(:,52); % y��x���ٶȼư�װ���,��λarcmin
pdUAyz = data(:,51); % y��z���ٶȼư�װ���,��λarcmin
pdUAzx = data(:,53); % z��y���ٶȼư�װ���,��λarcmin
pdUAzy = data(:,54); % z��x���ٶȼư�װ���,��λarcmin

time = (1:n)/3600;
% close all;
%%
figure;
subplot(2,2,1);
plot(time,phiE,time,phiN,time,phiU);
legend('phiE','phiN','phiU');
ylabel('��̬�����(arcmin)');
subplot(2,2,2);
plot(time,pphiE,time,pphiN,time,pphiU);
legend('pphiE','pphiN','pphiU');
ylabel('��̬�����(arcmin)');
subplot(2,2,3);
plot(time,dvE,time,dvN,time,dvU);
legend('dvE','dvN','dvU');
ylabel('�ٶ����(m/s)');
xlabel('ʱ��(h)');
subplot(2,2,4);
plot(time,pdvE,time,pdvN,time,pdvU);
legend('pdvE','pdvN','pdvU');
ylabel('�ٶ����(m/s)');
xlabel('ʱ��(h)');
%%
figure;
subplot(3,2,1);
plot(time,BGx,time,BGy,time,BGz);
legend('BGx','BGy','BGz');
ylabel('���ݳ�ֵƯ��(deg/h)');
% xlabel('ʱ��(s)');
subplot(3,2,3);
plot(time,dKGx,time,dKGy,time,dKGz);
legend('dKGx','dKGy','dKGz');
ylabel('���ݱ�����(ppm)');
% xlabel('ʱ��(s)');
subplot(3,2,5);
plot(time,dUGxy,time,dUGxz,time,dUGyz);
legend('dUGxy','dUGxz','dUGyz');
ylabel('���ݰ�װ���(arcmin)');
xlabel('ʱ��(h)');
subplot(3,2,2);
plot(time,BAx,time,BAy,time,BAz);
legend('BAx','BAy','BAz');
ylabel('���ٶȼ���ƫ(mg)');
% xlabel('ʱ��(s)');
subplot(3,2,4);
plot(time,dKAx,time,dKAy,time,dKAz);
legend('dKAx','dKAy','dKAz');
ylabel('���ٶȼƱ�����(ppm)');
% xlabel('ʱ��(s)');
subplot(3,2,6);
plot(time,dUAxy,time,dUAxz,time,dUAyx,time,dUAyz,time,dUAzx,time,dUAzy);
legend('dUAxy','dUAxz','dUAyx','dUAyz','dUAzx','dUAzy');
ylabel('���ٶȼư�װ���(arcmin)');
xlabel('ʱ��(h)');

%%
figure;
subplot(3,2,1);
plot(time,pBGx,time,pBGy,time,pBGz);
legend('pBGx','pBGy','pBGz');
ylabel('���ݳ�ֵƯ��(deg/h)');
% xlabel('ʱ��(s)');
subplot(3,2,2);
plot(time,pBAx,time,pBAy,time,pBAz);
legend('pBAx','pBAy','pBAz');
ylabel('���ٶȼ���ƫ(mg)');
% xlabel('ʱ��(s)');
subplot(3,2,3);
plot(time,pdKGx,time,pdKGy,time,pdKGz);
legend('pdKGx','pdKGy','pdKGz');
ylabel('���ݱ�����(ppm)');
% xlabel('ʱ��(s)');
subplot(3,2,5);
plot(time,pdUGxy,time,pdUGxz,time,pdUGyz);
legend('pdUGxy','pdUGxz','pdUGyz');
ylabel('���ݰ�װ���(arcmin)');
% xlabel('ʱ��(s)');
subplot(3,2,4);
plot(time,pdKAx,time,pdKAy,time,pdKAz);
legend('pdKAx','pdKAy','pdKAz');
ylabel('���ٶȼƱ�����(ppm)');
% xlabel('ʱ��(s)');
subplot(3,2,6);
plot(time,pdUAxy,time,pdUAxz,time,pdUAyx,time,pdUAyz,time,pdUAzx,time,pdUAzy);
legend('pdUAxy','pdUAxz','pdUAyx','pdUAyz','pdUAzx','pdUAzy');
ylabel('���ٶȼư�װ���(arcmin)');
xlabel('ʱ��(h)');
