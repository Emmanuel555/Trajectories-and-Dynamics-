clear; % clears all var
clc; % clears command window
data_angle = load("ph_no_gs_back_motor_angle.mat");
data_pos = load("ph_no_gs_back_motor_pos.mat");
% acceleration = load("acc_y_roll_deg.mat");
% f = load("flap_angle.mat");
time_att = transpose(data_angle.txyzangle(1,:));
time_pos = transpose(data_pos.txyzposrps(1,:));
pos_x = transpose(data_pos.txyzposrps(3,:));
pos_y = transpose(data_pos.txyzposrps(4,:));
pos_z = transpose(data_pos.txyzposrps(5,:));
% front motor cw, back motor acw
rps = -1 * transpose(data_pos.txyzposrps(6,:));
bod_pitch = transpose(data_angle.txyzangle(3,:));
bod_roll = transpose(data_angle.txyzangle(4,:));
bod_pitch = abs(bod_pitch);
smoothed_bod_roll = smoothdata(bod_roll,'gaussian',750);
smoothed_bod_pitch = smoothdata(bod_pitch,'gaussian',1000);
smoothed_rps = smoothdata(rps,'gaussian',750);

mean_pitch = mean(bod_pitch);
mean_roll = mean(abs(bod_roll));
mean_rps = mean(rps);

disp(mean_pitch)
disp(mean_roll)
disp(mean_rps)

% num = 0.0;
% N = size(time_pos);
% N = N(2);
% 
% 
% for i = N
%     disp ("lalala")
% %     euclidean = sqrt(power(pos_x(i),2) + power(pos_y(i),2) + power(pos_z(i),2));
% %     disp(euclidean)
% %     num = num + power((euclidean - 4),2); 
% end
% 
% %RMSE = sqrt(num/N(1,2));
% disp(num)





t = tiledlayout(3,1);
nexttile
plot(time_att,smoothed_bod_roll,'Color','b','linewidth',5)
xlabel('Time')
ylabel('Body roll in deg')
title('Body roll for position hold in clockwise direction','fontsize',14)
grid on

nexttile
plot(time_att,smoothed_bod_pitch,'Color','r','linewidth',5)
xlabel('Time')
ylabel('Body pitch in deg')
title('Body pitch for position hold in clockwise direction','fontsize',14)
grid on

nexttile
plot(time_pos,smoothed_rps,'Color','k','linewidth',5)
xlabel('Time')
ylabel('Rad/s')
title('Body RPS for clockwise direction','fontsize',14)
grid on

