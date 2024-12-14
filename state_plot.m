function []= state_plot(time, states, linestyle)
color_1 = 'r';
color_2 = 'b';
color_3 = 'g';
figure(1000)
title('delta states from reference')
subplot(4,1,1)
hold on
plot(time, states(:, 1), 'Color', color_1, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 2), 'Color', color_2, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 3), 'Color', color_3, 'LineWidth', 3, 'LineStyle', linestyle)
xlim([3, 13])

set(gca,'fontsize',16)
legend('pn', 'pe', 'pd')
title('position')
ylabel('(m)')
grid on

subplot(4,1,2)
hold on
plot(time, states(:, 4), 'Color', color_1, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 5), 'Color', color_2, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 6), 'Color', color_3, 'LineWidth', 3, 'LineStyle', linestyle)
xlim([3, 13])
set(gca,'fontsize',16)
legend('u', 'v', 'w');
title('translational rates')
ylabel('(m/s)')
grid on

subplot(4,1,3)
hold on
plot(time, states(:, 7).*(180/pi), 'Color', color_1, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 8).*(180/pi), 'Color', color_2, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 9).*(180/pi), 'Color', color_3, 'LineWidth', 3, 'LineStyle', linestyle)
xlim([3, 13])
set(gca,'fontsize',16)
legend('\phi', '\theta', '\psi')
title('attitude')
ylabel('(degrees)')
grid on

subplot(4,1,4)
hold on
plot(time, states(:, 10).*(180/pi), 'Color', color_1, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 11).*(180/pi), 'Color', color_2, 'LineWidth', 3, 'LineStyle', linestyle)
plot(time, states(:, 12).*(180/pi), 'Color', color_3, 'LineWidth', 3, 'LineStyle', linestyle)
xlim([3, 13])
set(gca,'fontsize',16)
legend('p', 'q', 'r')
title('angular rates')
ylabel('(degrees/s)')
xlabel('time (s)')
grid on
end