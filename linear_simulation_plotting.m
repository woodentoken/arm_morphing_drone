    % plot stat
    color_1 = [1 - (c-1)/(size(combination_length_roll,1)), 0, 0];
    color_2 = [0, 1 - (c-1)/(size(combination_length_roll,1)), 0];
    color_3 = [0, 0, 1 - (c-1)/(size(combination_length_roll,1))];


    input_color_1 = [1 - (c-1)/(size(combination_length_roll,1)), 0, 1];
    input_color_2 = [0, 1 - (c-1)/(size(combination_length_roll,1)), 1];
    input_color_3 = [1, 0, 1 - (c-1)/(size(combination_length_roll,1))];
    input_color_4 = [0, 1, 1 - (c-1)/(size(combination_length_roll,1))];

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

    output = sim("linear_system.slx");
    time = output.tout;

    linear_simulation_states = squeeze(output.yout{1}.Values.Data);
    linear_simulation_inputs = squeeze(output.yout{2}.Values.Data);
    
    figure(100)
    subplot(5,1,1)
    hold on
    plot(time, linear_simulation_states(:, 1), 'Color', color_1, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 2), 'Color', color_2, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 3), 'Color', color_3, 'LineWidth', 2)

    set(gca,'fontsize',16)
    legend('pn', 'pe', 'h')
    ylabel('position (m)')
    xlabel('time (s)')
    grid on

    subplot(5,1,2)
    hold on
    plot(time, linear_simulation_states(:, 4), 'Color', color_1, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 5), 'Color', color_2, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 6), 'Color', color_3, 'LineWidth', 2)
    set(gca,'fontsize',16)
    legend('u', 'v', 'w');
    ylabel('translational rates (m/s)')
    xlabel('time (s)')
    grid on

    subplot(5,1,3)
    hold on
    plot(time, linear_simulation_states(:, 7).*(180/pi), 'Color', color_1, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 8).*(180/pi), 'Color', color_2, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 9).*(180/pi), 'Color', color_3, 'LineWidth', 2 )
    set(gca,'fontsize',16)
    legend('phi', 'theta', 'psi')
    ylabel('angles (degrees)')
    xlabel('time (s)')
    grid on

    subplot(5,1,4)
    hold on
    plot(time, linear_simulation_states(:, 10).*(180/pi), 'Color', color_1, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 11).*(180/pi), 'Color', color_2, 'LineWidth', 2)
    plot(time, linear_simulation_states(:, 12).*(180/pi), 'Color', color_3, 'LineWidth', 2)
    set(gca,'fontsize',16)
    legend('p', 'q', 'r')
    ylabel('angular rates (degrees/s)')
    xlabel('time (s)')
    grid on

    subplot(5,1,5)
    hold on
    plot(time, linear_simulation_inputs(:, 1), 'Color', input_color_1, 'LineWidth', 2)
    plot(time, linear_simulation_inputs(:, 2), 'Color', input_color_2, 'LineWidth', 2)
    plot(time, linear_simulation_inputs(:, 3), 'Color', input_color_3, 'LineWidth', 2)
    plot(time, linear_simulation_inputs(:, 4), 'Color', input_color_4, 'LineWidth', 2)
    set(gca,'fontsize',16)
    legend('d_f', 'd_r', 'd_b', 'd_l')
    ylabel('delta inputs from trim')
    xlabel('time (s)')
    grid on

%% Eigenvalue plotting
    eigvals = eig(linsys_cl);

    figure(101)
    hold on
    grid on
    plot(eigvals, 'x', 'Color', color_1, 'MarkerSize', 15, 'LineWidth', 3);
    xlabel('real')
    ylabel('imaginary')
    xlim([-9 0])
    ylim([-3 3])
    legend()