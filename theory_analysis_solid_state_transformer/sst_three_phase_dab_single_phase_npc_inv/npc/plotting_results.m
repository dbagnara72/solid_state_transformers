close all
clc

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
time_offset = 0.0;
t1c = time_tc_sim(end) - Nc*tc/1000 - time_offset;
t2c = time_tc_sim(end) - time_offset;
t1s = time_ts_dab_modA_sim(end) - Ns_dab*ts_dab/2;
t2s = time_ts_dab_modA_sim(end);
t3s = time_ts_dab_modA_sim(end) - Ns_dab*ts_dab/10;
t4s = time_ts_dab_modA_sim(end);
t5s = time_ts_dab_modA_sim(end) - Ns_dab*ts_dab/20;
t6s = time_ts_dab_modA_sim(end);
t3c = time_tc_sim(end) - Nc*tc/200 - time_offset;
t4c = time_tc_sim(end) - time_offset;
t5c = time_tc_sim(end) - Nc*tc/600 - time_offset;
t6c = time_tc_sim(end) - time_offset;
fontsize_plotting = 14;

figure(1);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_output_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1000 500]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,dab_voltage_output_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[500 1650]);
hold off
title('DAB output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2}^{dc}$','$u_{2}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_input_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-325 -200]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,dab_voltage_input_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[1275 1375]);
hold off
title('DAB input current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{dc}$','$u_{1}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_ui_output_input','-depsc');
movefile('dab_ui_output_input.eps', 'figures');

figure(2);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i2_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-500 500]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,2),'--','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
hold off
title('Transformer-DAB: secondary side current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2u}^{ac}$','$u_{2u}^{ac}$','$u_{2v}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-500 500]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,2),'--','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
hold off
title('Transformer-DAB: primary side current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1u}^{ac}$','$u_{1u}^{ac}$','$u_{1v}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_transformer_voltage_current','-depsc');
movefile('dab_transformer_voltage_current.eps', 'figures');

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,current_battery_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-600 -500]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,voltage_battery_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[1200 1400]);
hold off
title('DC grid (battery) current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{g}^{dc}$','$u_{g}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ac_grid_current_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-2000 2000]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,ac_grid_voltage_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
hold off
title('AC grid current and voltage - phase U','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{gu}^{ac}$','$u_{gu}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dc_ac_grid_voltage_current','-depsc');
movefile('dc_ac_grid_voltage_current.eps', 'figures');


figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,is_inv_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-1600 1600]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,us_inv_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1000 1000]);
hold off
title('Single Phase Inverter: output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{s}^{ac}$','$u_{s}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,inverter_device_data_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[300 600]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,inverter_device_data_modA_sim(:,6),'-','LineWidth',tratto2,'Color',colore2);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[100 2000]);
hold off
title('Single Phase Inverter: Q1/Q2 devices power loss','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{Q_1}$','$p_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('single_phase_inverter_performance','-depsc');
movefile('single_phase_inverter_performance.eps', 'figures');

figure;
subplot 211
plot(time_tc_sim,inverter_device_data_modA_sim(:,2),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1000]);
title('Single Phase Inverter: Q1 voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
subplot 212
plot(time_tc_sim,inverter_device_data_modA_sim(:,3),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1600 1000]);
legend('$i_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
title('Single Phase Inverter: Q1 current','Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('single_phase_inverter_Q1','-depsc');
movefile('single_phase_inverter_Q1.eps', 'figures');

figure;
subplot 211
plot(time_tc_sim,inverter_device_data_modA_sim(:,7),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1000]);
title('Single Phase Inverter: Q2 voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
grid on
set(gca,'xlim',[t5s t6s]);
subplot 212
plot(time_tc_sim,inverter_device_data_modA_sim(:,8),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1600 1600]);
legend('$i_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
title('Single Phase Inverter: Q2 current','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('single_phase_inverter_Q2','-depsc');
movefile('single_phase_inverter_Q2.eps', 'figures')

figure;
subplot 311
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[100 175]);
set(gca,'xlim',[t1s t2s]);
title('DAB primary side: Q1 power loss','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 312
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,2),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1750]);
legend('$u_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB primary side: Q1 voltage','Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 313
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,3),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-400 200]);
legend('$i_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB primary side: Q1 current','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('DAB_primary_side_Q1','-depsc');
movefile('DAB_primary_side_Q1.eps', 'figures')


figure;
subplot 311
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,6),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[100 175]);
set(gca,'xlim',[t1s t2s]);
title('DAB primary side: Q2 power loss','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 312
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,7),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1750]);
legend('$u_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB primary side: Q2 voltage','Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 313
plot(time_tc_sim,inverter_1_dab_devices_data_modA_sim(:,8),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-400 200]);
legend('$i_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB primary side: Q2 current','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('DAB_primary_side_Q2','-depsc');
movefile('DAB_primary_side_Q2.eps', 'figures')

figure;
subplot 311
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[225 325]);
set(gca,'xlim',[t1s t2s]);
title('DAB secondary side: Q1 power loss','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 312
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,2),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1750]);
legend('$u_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB secondary side: Q1 voltage','Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 313
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,3),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-200 400]);
legend('$i_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB secondary side: Q1 current','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('DAB_secondary_side_Q1','-depsc');
movefile('DAB_secondary_side_Q1.eps', 'figures')


figure(10);
subplot 311
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,6),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[225 325]);
set(gca,'xlim',[t1s t2s]);
title('DAB secondary side: Q2 power loss','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 312
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,7),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-50 1750]);
legend('$u_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB secondary side: Q2 voltage','Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 313
plot(time_tc_sim,inverter_2_dab_devices_data_modA_sim(:,8),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-200 400]);
legend('$i_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5c t6c]);
title('DAB secondary side: Q2 current','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('DAB_secondary_side_Q2','-depsc');
movefile('DAB_secondary_side_Q2.eps', 'figures')