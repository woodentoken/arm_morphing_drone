
function [] = input_plot(time, inputs, linestyle)
%INPUT_PLOT Summary of this function goes here
%   Detailed explanation goes here
input_color_1 = 1.5.*[20/255, 43/255, 109/255];
input_color_2 = 2.*[48/255, 30/255, 30/255];
input_color_3 = [147/255, 138/255, 25/255];
input_color_4 = [44/255, 98/255, 50/255];

figure(1001)
subplot(4,1,1)
hold on
plot(time, inputs(:, 2), 'Color', input_color_1, 'LineWidth', 2, 'LineStyle', linestyle)
set(gca,'fontsize',16)
title('\Delta \delta_f')
legend('one arm', 'two arm')
grid on
ylim([-3,3])

subplot(4,1,2)
hold on
plot(time, inputs(:, 2), 'Color', input_color_2, 'LineWidth', 2, 'LineStyle', linestyle)
set(gca,'fontsize',16)
legend('one arm', 'two arm')
title('\Delta \delta_r')
grid on
ylim([-3,3])

subplot(4,1,3)
hold on
plot(time, inputs(:, 3), 'Color', input_color_3, 'LineWidth', 2, 'LineStyle', linestyle)
set(gca,'fontsize',16)
legend('one arm', 'two arm')
title('\Delta \delta_b')
ylim([-3,3])
grid on

subplot(4,1,4)
hold on
plot(time, inputs(:, 4), 'Color', input_color_4, 'LineWidth', 2, 'LineStyle', linestyle)
set(gca,'fontsize',16)
legend('one arm', 'two arm')
title('\Delta \delta_l')
xlabel('time (s)')
grid on
ylim([-3,3])
end