format short
close all; clearvars; clc
syms pn_dot pe_dot h_dot u_dot v_dot w_dot phi_dot theta_dot psi_dot p_dot q_dot r_dot
syms pn pe h u v w phi theta psi p q r
syms l_f l_r l_b l_l k1 k2 delta_f delta_r delta_l delta_b g


% Function to calculate error from target state
function error = non_linear_trim(trim_guess, J_matrix_body, CG_matrix, m_tot, config, target_state_dot)
state_trim = trim_guess(1:12);
control_trim = trim_guess(13:16);
state_dot = calculate_quadcopter_eom(state_trim, control_trim, J_matrix_body, CG_matrix, m_tot, config);

error = state_dot - target_state_dot;
error = double(error);
end

% Function to calculate the outputs
function [trim_dot_output, control_input_at_trim] = non_linear_trim_master_calculation(target_state_dot_out, config) % combination_length_roll is actually arm length configuratino
gravity = 9.81;

% Calculate CG matrix, J and m_tot
[CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(config);

% Initial guess for trim states and inputs
x_state_guess = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
u_input_guess = [m_tot*gravity/4; m_tot*gravity/4; m_tot*gravity/4; m_tot*gravity/4];
trim_guess = [x_state_guess; u_input_guess];

% Solver for trim conditions by iteration
option = optimoptions('fsolve', 'Display', 'Iter', 'MaxFunctionEvaluations', 1e6);
solving_function = @(trim_guess) non_linear_trim(trim_guess, J_matrix_body, CG_matrix, m_tot, config, target_state_dot_out);
[trim_results, ~] = fsolve(solving_function, trim_guess, option);

trim_dot_output = trim_results(1:12);
control_input_at_trim = trim_results(13:16);
end

%% Main code start at:
% Define the configuration:
% load_constant_values

configs = [
    [0.3, 0.3, -0.3, -0.3];
    [0.2, 0.4, -0.2, -0.2];
    [0.4, 0.4, -0.2, -0.2];
    ];
T = generate_trajectory();

output_set = [];
for i=1:size(configs,1)
    config = configs(i, :)

    % config = [linspace(0.3, 1, 1000)' linspace(0.3, 1, 1000)' linspace(0.3, 1, 1000)' linspace(0.3, 0.3, 1000)']

    % Define your target trim state dot:
    target_state_dot_out = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

    % Run non-linear trim condition solver
    % Input: Config
    %        Target Trim State Dot
    % Output: x_trim
    %         control_input_trim
    [x_trim, u_trim] = non_linear_trim_master_calculation(target_state_dot_out, config);

    % Define state and control vectors symbolically
    x_sym = [pn; pe; h; u; v; w; phi; theta; psi; p; q; r];
    u_sym = [delta_f; delta_r; delta_b; delta_l];

    reference_config = config;

    % Calculate CG matrix, J and m_tot
    [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(reference_config);
    % Calculate equation of motion
    equation_of_motion = calculate_quadcopter_eom(x_sym, u_sym, J_matrix_body, CG_matrix, m_tot, reference_config);

    jaco_about_x = jacobian(equation_of_motion, x_sym);
    jaco_about_u = jacobian(equation_of_motion, u_sym);

    A_matrix = subs(jaco_about_x, x_sym, x_trim);
    B_matrix = subs(jaco_about_u, u_sym, u_trim);
    C_matrix = eye(12);
    D_matrix = 0;
    linsys = ss(double(A_matrix), double(B_matrix),C_matrix,D_matrix);

    Q = eye(12);
    Q(1:3, 1:3) = eye(3,3);
    Q(4:6,4:6) = eye(3,3);
    Q(7:9, 7:9) = eye(3,3);
    Q(10:12,10:12) = 5.*eye(3,3);
    R = eye(4);

    [K_lqr, P, S] = lqr(linsys, Q, R);

        load("K_ref.mat", "K_lqr")
    simIn = Simulink.SimulationInput("simulink/linear_system.slx"); %create object
    simIn = simIn.setVariable("my_Parameter",2); %example of setting a parameter override
    
    output = sim(simIn);
    output_set = [output_set; output];
    plot_simulation_results(output, i, size(configs, 1));


    figure(666)
    hold on
    states = squeeze(output.yout{1}.Values.Data)';
    states_attitude = states(:,7:9).*180/pi;
    time = squeeze(output.tout)';
    plot(time, states_attitude)
end

% save("K_ref", "K_lqr")

output_set;
time = squeeze(output_set(1).tout)';

baseline_states = squeeze(output_set(1).yout{1}.Values.Data)';
baseline_input = squeeze(output_set(1).yout{2}.Values.Data)';

one_arm_states = squeeze(output_set(2).yout{1}.Values.Data)';
one_arm_input = squeeze(output_set(2).yout{2}.Values.Data)';

two_arm_states = squeeze(output_set(3).yout{1}.Values.Data)';
two_arm_input = squeeze(output_set(3).yout{2}.Values.Data)';

input_one_arm = one_arm_input - baseline_input;
input_two_arm = two_arm_input - baseline_input;

states_one_arm = one_arm_states - baseline_states;
% states_one_arm = states_one_arm - ones(size(states_one_arm,1),1).*states_one_arm(1,:);

states_two_arm = two_arm_states- baseline_states;
% states_two_arm = states_two_arm - ones(size(states_two_arm,1),1).*states_two_arm(1,:);

% nonlinear_simulation_states = states_two_arm;

state_plot(time, states_one_arm, '-');
state_plot(time, states_two_arm, '-.');

input_plot(time, input_one_arm, '-')
input_plot(time, input_two_arm, '-.')

