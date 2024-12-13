format long
syms pn_dot pe_dot h_dot u_dot v_dot w_dot phi_dot theta_dot psi_dot p_dot q_dot r_dot
syms pn pe h u v w phi theta psi p q r 
syms l_f l_r l_b l_l k1 k2 delta_f delta_r delta_l delta_b g 
close all

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

% Main code start at:
% Define the configuration:
config = [0.4, 0.4, -0.2, -0.2];

% Define your target trim state dot:
target_state_dot_out = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% Run non-linear trim condition solver
% Input: Config 
%        Target Trim State Dot
% Output: x_trim
%         control_input_trim
[x_trim, control_input_trim] = non_linear_trim_master_calculation(target_state_dot_out, config);

disp(x_trim)
disp(control_input_trim)