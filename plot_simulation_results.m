function [] = plot_simulation_results(sim_out, sequence_index, sequence_len)
color_1 = [1 - (sequence_index-1)/(1.2*sequence_len), 0, 0];
color_2 = [0, 1 - (sequence_index-1)/(1.2*sequence_len), 0];
color_3 = [0, 0, 1 - (sequence_index-1)/(1.2*sequence_len)];
color_error = [0.8 - (sequence_index-1)/(sequence_len), 0.8 - (sequence_index-1)/(sequence_len), 0.8 - (sequence_index-1)/(sequence_len)];
% color_error = [0,0,0];

input_color_1 = [1 - (sequence_index-1)/(1.2*sequence_len), 0, 0];
input_color_2 = [0, 1 - (sequence_index-1)/(1.2*sequence_len), 0];
input_color_3 = [0, 0, 1 - (sequence_index-1)/(1.2*sequence_len)];
input_color_4 = [(sequence_index-1)/(2*sequence_len), 0, 0.5 - (sequence_index-1)/(2*sequence_len)];

time = sim_out.tout;

%% reference trajectory
ref_translation = squeeze(sim_out.yout{3}.Values.Data)';
ref_rotation =  180/pi .* squeeze(sim_out.yout{4}.Values.Data)';

%% nonlinear response
nonlinear_simulation_states = squeeze(sim_out.yout{1}.Values.Data)';
nonlinear_simulation_inputs = squeeze(sim_out.yout{2}.Values.Data)';

% figure(6)
% hold on
% grid on
% ylabel("normalized error (nonlinear)");
% xlabel("time (s)");
% plot(time, normalized_error_nonlinear, 'Color', color_error, 'LineWidth', 2);

figure(101)
subplot(5,1,1)
hold on
plot(time, nonlinear_simulation_states(:, 1), 'Color', color_1, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 2), 'Color', color_2, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 3), 'Color', color_3, 'LineWidth', 2)
% plot(time, ref_translation, 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2)

set(gca,'fontsize',16)
legend('pn', 'pe', 'pd')
title('position')
ylabel('(m)')
% xlabel('time (s)')
grid on

subplot(5,1,2)
hold on
plot(time, nonlinear_simulation_states(:, 4), 'Color', color_1, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 5), 'Color', color_2, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 6), 'Color', color_3, 'LineWidth', 2)
set(gca,'fontsize',16)
legend('u', 'v', 'w');
title('translational rates')
ylabel('(m/s)')
% ylim([-5, 10]);
% xlabel('time (s)')
grid on

subplot(5,1,3)
hold on
plot(time, nonlinear_simulation_states(:, 7).*(180/pi), 'Color', color_1, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 8).*(180/pi), 'Color', color_2, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 9).*(180/pi), 'Color', color_3, 'LineWidth', 2)
% plot(time, ref_rotation, 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2)
set(gca,'fontsize',16)
legend('\phi', '\theta', '\psi')
title('attitude')
% ylim([-90, 90]);
ylabel('(degrees)')
% xlabel('time (s)')
grid on

subplot(5,1,4)
hold on
plot(time, nonlinear_simulation_states(:, 10).*(180/pi), 'Color', color_1, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 11).*(180/pi), 'Color', color_2, 'LineWidth', 2)
plot(time, nonlinear_simulation_states(:, 12).*(180/pi), 'Color', color_3, 'LineWidth', 2)
set(gca,'fontsize',16)
legend('p', 'q', 'r')
title('angular rates')
ylabel('(degrees/s)')
% xlabel('time (s)')
grid on

subplot(5,1,5)
hold on
plot(time, nonlinear_simulation_inputs(:, 1), 'Color', input_color_1, 'LineWidth', 2)
plot(time, nonlinear_simulation_inputs(:, 2), 'Color', input_color_2, 'LineWidth', 2)
plot(time, nonlinear_simulation_inputs(:, 3), 'Color', input_color_3, 'LineWidth', 2)
plot(time, nonlinear_simulation_inputs(:, 4), 'Color', input_color_4, 'LineWidth', 2)
yline(0)
set(gca,'fontsize',16)
legend('d_f', 'd_r', 'd_b', 'd_l')
title('\Delta input from trim')
xlabel('time (s)')
grid on
axis tight