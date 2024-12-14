    % plot stat
    color_1 = [1 - (c-1)/(1.2*size(combination_length_roll,1)), 0, 0];
    color_2 = [0, 1 - (c-1)/(1.2*size(combination_length_roll,1)), 0];
    color_3 = [0, 0, 1 - (c-1)/(1.2*size(combination_length_roll,1))];
    color_error = [0.8 - (c-1)/(size(combination_length_roll,1)), 0.8 - (c-1)/(size(combination_length_roll,1)), 0.8 - (c-1)/(size(combination_length_roll,1))];
    % color_error = [0,0,0];
        
    input_color_1 = [1 - (c-1)/(1.2*size(combination_length_roll,1)), 0, 0];
    input_color_2 = [0, 1 - (c-1)/(1.2*size(combination_length_roll,1)), 0];
    input_color_3 = [0, 0, 1 - (c-1)/(1.2*size(combination_length_roll,1))];
    input_color_4 = [(c-1)/(2*size(combination_length_roll,1)), 0, 0.5 - (c-1)/(2*size(combination_length_roll,1))];


    % color_1 = 'r';
    % color_2 = 'b';
    % color_3 = 'g';
    % color_error = 'm';
    % 
    % input_color_1 = 'b';
    % input_color_2 = 'r';
    % input_color_3 = 'c';
    % input_color_4 = 'm';

    initial_condition = [0,0,0, 0,0,0, 0,0,0, 0,0,0];

    num_times = 200;
    inputs = zeros(num_times,4); % zero input case, this is only initial condition response!
    time = linspace(0,5,num_times);
    inputs(:,2) = sin(time);
    % inputs(:,4) = 1;

    % lp = lsimplot(linsys_cl, inputs, time, initial_condition_state, plotopts);
    % linear_simulation = lsim(linsys_cl, inputs, time, initial_condition);

    output = sim("simulink/linear_system.slx");
    time = output.tout;

    %% linear response
    linear_simulation_states = squeeze(output.yout{1}.Values.Data)';
    linear_simulation_inputs = squeeze(output.yout{2}.Values.Data)';

    ref_translation = squeeze(output.yout{5}.Values.Data)';
    ref_rotation = squeeze(output.yout{6}.Values.Data)';
    ref_rotation = 180/pi .* ref_rotation;

    % figure(5)
    % hold on
    % grid on
    % ylabel("normalized error (linear)");
    % xlabel("time (s)");
    % plot(time, normalized_error_linear, 'Color', color_error, 'LineWidth', 3);
    figure(100)
    subplot(5,1,1)
    hold on
    plot(time, linear_simulation_states(:, 1), 'Color', color_1, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 2), 'Color', color_2, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 3), 'Color', color_3, 'LineWidth', 3)
    plot(time, ref_translation, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 3)

    set(gca,'fontsize',16)
    legend('pn', 'pe', 'h')
    title('position')
    ylabel('(m)')
    % xlabel('time (s)')
    grid on

    subplot(5,1,2)
    hold on
    plot(time, linear_simulation_states(:, 4), 'Color', color_1, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 5), 'Color', color_2, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 6), 'Color', color_3, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('u', 'v', 'w');
    title('translational rates')
    % ylim([-5, 10]);
    ylabel('(m/s)')

    % xlabel('time (s)')
    grid on

    subplot(5,1,3)
    hold on
    plot(time, linear_simulation_states(:, 7).*(180/pi), 'Color', color_1, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 8).*(180/pi), 'Color', color_2, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 9).*(180/pi), 'Color', color_3, 'LineWidth', 3)
    plot(time, ref_rotation, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 5)
    set(gca,'fontsize',16)
    legend('\phi', '\theta', '\psi')
    title('attitude')
    ylabel('(degrees)')
    % ylim([-90, 90]);
    % xlabel('time (s)')
    grid on

    subplot(5,1,4)
    hold on
    plot(time, linear_simulation_states(:, 10).*(180/pi), 'Color', color_1, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 11).*(180/pi), 'Color', color_2, 'LineWidth', 3)
    plot(time, linear_simulation_states(:, 12).*(180/pi), 'Color', color_3, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('p', 'q', 'r')
    title('angular rates')
    ylabel('(degrees/s)')
    % xlabel('time (s)')
    grid on

    subplot(5,1,5)
    hold on
    plot(time, linear_simulation_inputs(:, 1), 'Color', input_color_1, 'LineWidth', 3)
    plot(time, linear_simulation_inputs(:, 2), 'Color', input_color_2, 'LineWidth', 3)
    plot(time, linear_simulation_inputs(:, 3), 'Color', input_color_3, 'LineWidth', 3)
    plot(time, linear_simulation_inputs(:, 4), 'Color', input_color_4, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('d_f', 'd_r', 'd_b', 'd_l')
    title('\Delta input from trim')
    xlabel('time (s)')
    grid on

%% nonlinear response
    nonlinear_simulation_states = squeeze(output.yout{3}.Values.Data)';
    nonlinear_simulation_inputs = squeeze(output.yout{4}.Values.Data)';

    % figure(6)
    % hold on
    % grid on
    % ylabel("normalized error (nonlinear)");
    % xlabel("time (s)");
    % plot(time, normalized_error_nonlinear, 'Color', color_error, 'LineWidth', 3);

    figure(101)
    subplot(5,1,1)
    hold on
    plot(time, nonlinear_simulation_states(:, 1), 'Color', color_1, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 2), 'Color', color_2, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 3), 'Color', color_3, 'LineWidth', 3)
    plot(time, ref_translation, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 3)


    set(gca,'fontsize',16)
    legend('pn', 'pe', 'h')
    title('position')
    ylabel('(m)')
    % xlabel('time (s)')
    grid on

    subplot(5,1,2)
    hold on
    plot(time, nonlinear_simulation_states(:, 4), 'Color', color_1, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 5), 'Color', color_2, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 6), 'Color', color_3, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('u', 'v', 'w');
    title('translational rates')
    ylabel('(m/s)')
    % ylim([-5, 10]);
    % xlabel('time (s)')
    grid on

    subplot(5,1,3)
    hold on
    plot(time, nonlinear_simulation_states(:, 7).*(180/pi), 'Color', color_1, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 8).*(180/pi), 'Color', color_2, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 9).*(180/pi), 'Color', color_3, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('\phi', '\theta', '\psi')
    title('attitude')
    % ylim([-90, 90]);
    ylabel('(degrees)')
    % xlabel('time (s)')
    grid on

    subplot(5,1,4)
    hold on
    plot(time, nonlinear_simulation_states(:, 10).*(180/pi), 'Color', color_1, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 11).*(180/pi), 'Color', color_2, 'LineWidth', 3)
    plot(time, nonlinear_simulation_states(:, 12).*(180/pi), 'Color', color_3, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('p', 'q', 'r')
    title('angular rates')
    ylabel('(degrees/s)')
    % xlabel('time (s)')
    grid on

    subplot(5,1,5)
    hold on
    plot(time, nonlinear_simulation_inputs(:, 1), 'Color', input_color_1, 'LineWidth', 3)
    plot(time, nonlinear_simulation_inputs(:, 2), 'Color', input_color_2, 'LineWidth', 3)
    plot(time, nonlinear_simulation_inputs(:, 3), 'Color', input_color_3, 'LineWidth', 3)
    plot(time, nonlinear_simulation_inputs(:, 4), 'Color', input_color_4, 'LineWidth', 3)
    set(gca,'fontsize',16)
    legend('d_f', 'd_r', 'd_b', 'd_l')
    title('\Delta input from trim')
    xlabel('time (s)')
    grid on
    axis tight