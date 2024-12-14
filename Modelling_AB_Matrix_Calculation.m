format long
syms pn_dot pe_dot h_dot u_dot v_dot w_dot phi_dot theta_dot psi_dot p_dot q_dot r_dot
syms pn pe h u v w phi theta psi p q r 
syms l_f l_r l_b l_l k1 k2 delta_f delta_r delta_l delta_b g 
close all

%% Interpolation of thrust from dataset
function thrust = get_thrust_from_throttle(throttle_percentage, throttle_table, thrust_table)
    % Inputs:
    %   throttle_percentage: The throttle percentage input (0 to 100%)
    %   throttle_table: Vector of throttle percentages from the lookup table
    %   thrust_table: Vector of corresponding thrust values (in grams)
    % Output:
    %   thrust: The interpolated thrust corresponding to the input throttle percentage

    % Ensure the throttle percentage is within the bounds
    throttle_percentage = max(min(throttle_percentage, max(throttle_table)), min(throttle_table));
    
    % Interpolate to find the thrust
    thrust = interp1(throttle_table, thrust_table, throttle_percentage, 'linear');
end

% clearvars; close all; clc;

%% Script
% Define the parameters
% Arm length
max_arm_length = 0.2;
min_arm_length = 0.4;
number_of_discretization = 3;
arm_length_num = linspace(min_arm_length, max_arm_length, number_of_discretization);

% Initialize combination group
combination_length = [];

ID = 0;
% Creating all possible combinations by different length
for i = 1:length(arm_length_num)
    l_f_abs = arm_length_num(i);
    for j = 1:length(arm_length_num)
        l_r_abs = arm_length_num(j);
        for k = 1:length(arm_length_num)
            l_b_abs = arm_length_num(k);
            for z = 1:length(arm_length_num)
                l_l_abs = arm_length_num(z); 
                ID = ID+1;
                % Append the current combination with correct signs
                combination_length = [combination_length; ...
                    l_f_abs, l_r_abs, -l_b_abs, -l_l_abs, ID];
            end
        end
    end
end

% % Define the throttle percentages and corresponding thrusts
% throttle_table = [50, 62.5, 75, 88, 100];  % Throttle percentages (%)
% thrust_table = [666, 832.5, 999, 1165.5, 1332];  % Single thrust (g)
% initial_condition_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% Specify the folder name
output_folder = 'Modelling_AB_results';

% Create the folder in the current working directory
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

% asymmetric two arm
% combination_length_roll = [    
%     [0.4, 0.4, -0.2, -0.2];
%     [0.3, 0.3, -0.3, -0.3];
%     [0.3, 0.3, -0.3, -0.3];
%     ];

% asymmetric one arm
% combination_length_roll = [   
    % [0.2, 0.4, -0.2, -0.2];
    % [0.4, 0.2, -0.4, -0.4];
    % [0.4, 0.4, -0.3, -0.3];
    % ];

% inline
% combination_length_roll = [    
%     [0.2, 0.4, -0.2, -0.4];
%     [0.4, 0.2, -0.4, -0.2];
%     [0.3, 0.3, -0.3, -0.3];
%     ];

%% baseline
combination_length_roll = [    
    [0.3, 0.3, -0.3, -0.3];
    ];

% combination_length_roll =[    
%     [0.2 0.2 -0.2 -0.2 1];
%     [0.3 0.3 -0.3 -0.3 2];
%     [0.4 0.4 -0.4 -0.4 3];
%     ];

% all configs
% combination_length_roll = [ 
%     % baseline
%     [0.3, 0.3, -0.3, -0.3];
% 
%     % symmetric extension
%     [0.2, 0.2, -0.2, -0.2];
%     [0.4, 0.4, -0.4, -0.4];
% 
%     % one arm
%     [0.2, 0.4, -0.2, -0.2];
%     [0.4, 0.2, -0.4, -0.4];
% 
%     % inline
%     [0.2, 0.4, -0.2, -0.4];
%     [0.4, 0.2, -0.4, -0.2];
% 
%     % two arm
%     [0.4, 0.4, -0.2, -0.2];
%     ];

% loop over the selected combinations
entries = [];
K_ref = load("K_reference.mat");

for c = 1:size(combination_length_roll, 1)  % Use size for correct row count
    l_f = combination_length_roll(c, 1);    % Front arm length
    l_r = combination_length_roll(c, 2);    % Right arm length
    l_b = combination_length_roll(c, 3);    % Back arm length
    l_l = combination_length_roll(c, 4);    % Left arm length
    config = [l_f, l_r, l_b, l_l]; % defines the current config

    % Define state and control vectors symbolically
    control_matrix_x = [pn; pe; h; u; v; w; phi; theta; psi; p; q; r];
    control_matrix_u = [delta_f; delta_r; delta_b; delta_l]; 
    
    % Calculate CG matrix, J and m_tot
    [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(config);
    
    % Calculate equation of motion
    equation_of_motion = calculate_quadcopter_eom(control_matrix_x, control_matrix_u, J_matrix_body, CG_matrix, m_tot, config);

    sympref('FloatingPointOutput',true);
    vpa(equation_of_motion);
    
    %% Linearization
    % Jacobian Matrix:
    jaco_about_x = jacobian(equation_of_motion, control_matrix_x);
    jaco_about_u = jacobian(equation_of_motion, control_matrix_u);
    
    % Initial Conditions:
    syms psi_zero pn_zero pe_zero h_zero
    states = [pn pe h   u v w   phi theta psi   p q r];
    inputs = [delta_f delta_r delta_b delta_l];
    trim_state = [0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0];
    A_matrix = subs(jaco_about_x, states, trim_state);

    % calculate the trim
    balance_equation = [jaco_about_u(6, :); jaco_about_u(10:12, :)];
    desired_force_torque = [(m_tot*g)/k1; 0; 0; 0]; 
    trim_inputs = inv(balance_equation)*desired_force_torque;

    B_matrix = subs(jaco_about_u, inputs, trim_inputs');
    % B_matrix = subs(jaco_about_u, inputs, [(m_tot*g)/(k1*4), (m_tot*g)/(k1*4), (m_tot*g)/(k1*4), (m_tot*g)/(k1*4)]);

    A = double(A_matrix);
    B = double(B_matrix);
    C = eye(size(A));
    D = 0;

    linsys = ss(A,B,C,D);
    linsys.OutputName = {'pn', 'pe', 'h', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'};
    linsys.InputName = {'delta_f', 'delta_r', 'delta_b', 'delta_l'};

    %% sample LQR controller
    controllability = rank(ctrb(A,B));
    if controllability < size(A,1)       
        disp(["controllability matrix is not full rank: ", num2str(controllability)]);    end

    Q = 1.*eye(size(A));
    Q(3,3) = 5;
    R = 50.*eye(size(B,2));
    [K, S, P] = lqr(linsys, Q, R);

    linsys_cl = ss(A-B*K, B, C, D);

    linsys_cl.OutputName = {'pn', 'pe', 'h', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'};
    linsys_cl.InputName = {'delta_f', 'delta_r', 'delta_b', 'delta_l'};

    % ode45(@(t,x), nleom(t, x, K, Anl, Bnl, Hnl), t_a, x0);

    % plot the linear simulation results
    linear_simulation_plotting

    % Round arm lengths to integers for file naming convention
    l_f_int = round(l_f * 100);
    l_r_int = round(l_r * 100);
    l_b_int = round(l_b * 100);
    l_l_int = round(l_l * 100);

    % Generate unique filename with integers
    config_name = sprintf('ID%d_f%d_r%d_b%d_l%d', c, l_f_int, l_r_int, l_b_int, l_l_int);
    filename = fullfile(output_folder, sprintf('%s.mat', config_name));
    save(filename, 'config', 'A', 'B');
end

disp('Saved to Modelling_AB_results folder');