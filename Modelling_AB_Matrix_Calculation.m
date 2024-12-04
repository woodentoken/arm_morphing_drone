format long
syms pn_dot pe_dot h_dot u_dot v_dot w_dot phi_dot theta_dot psi_dot p_dot q_dot r_dot
syms pn pe h u v w phi theta psi p q r 
syms l_f l_r l_b l_l k1 k2 delta_f delta_r delta_l delta_b g 

% Interpolation of thrust from dataset
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

% Calcultate CG location and mass moment of inertia of the quadcopter J_matrix_body
function [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(l_f, l_r, l_b, l_l, t_motor, t_prop, d_cube, m_motor, m_prop, m_arm, m_cube, r_motor, r_prop)
% inputs:     Arm_lengths: (m)
%               l_f, 
%               l_r, 
%               l_b,
%               l_l
%             Thickness of individul components: (m)
%               t_motor, 
%               t_propeller,
%               d_cube.
%             Radius of cylinders: (m)
%               r_motor; 
%               r_prop.
%             mass of individul components: (kg)
%               m_motor, 
%               m_propeller, 
%               m_arm,
%               m_body.

% Outputs: CG location in xy:(m)
%               CG_matrix = [X_cg, Y_cg]
%          Mass moment of inertia matrix:(kg * m^2)
%               J_matrix_body = [ J_XX , -J_XY, -J_XZ;
%                                -J_XY ,  J_YY, -J_YZ;
%                                -J_XZ , -J_YZ,  J_ZZ]

% Assumption: 
% 1. CG in z-axis is not changing; 
% 2. Spininig props as a disk;
% 3. The disk has no thickness;
% 4. All the length l_b, l_f, l_l, l_r are all treat as signed
% +y = right rotor
% +x = front rotor
% +z = point down

    % Total mass
    m_tot = 4 * (m_motor + m_prop + m_arm) + m_cube;
    
    % CG location in x,y
    % X-axis CG calculation
    numerator_X = (m_motor + m_prop) * l_f + m_arm * (l_f / 2) + (m_motor + m_prop) * l_b + m_arm * (l_b / 2);
    X_cg = numerator_X / m_tot;
    % Y-axis CG calculation
    numerator_Y = (m_motor + m_prop) * l_r + m_arm * (l_r / 2) + (m_motor + m_prop) * l_l + m_arm * (l_l / 2);
    Y_cg = numerator_Y / m_tot;
    
    d_f_motor_prop = sqrt((l_f-X_cg)^2+Y_cg^2);
    d_b_motor_prop = sqrt((l_b-X_cg)^2+Y_cg^2);
    d_r_motor_prop = sqrt((l_r-Y_cg)^2+X_cg^2);
    d_l_motor_prop = sqrt((l_l-Y_cg)^2+X_cg^2);
    
    d_f_arm = sqrt((0.5*l_f-X_cg)^2+Y_cg^2);
    d_b_arm = sqrt((0.5*l_b-X_cg)^2+Y_cg^2);
    d_r_arm = sqrt((0.5*l_r-Y_cg)^2+X_cg^2);
    d_l_arm = sqrt((0.5*l_l-Y_cg)^2+X_cg^2);
    
    d_body = sqrt(X_cg^2 + Y_cg^2);
    
    % Calculating J_y
    J_y_front_motor = m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (l_f-X_cg)^2);
    J_y_front_prop  = m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (l_f-X_cg)^2);
    J_y_front_arm   = m_arm * ((1/12)*l_f^2 + (l_f/2-X_cg)^2);
    
    J_y_back_motor = m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (l_b-X_cg)^2);
    J_y_back_prop  = m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (l_b-X_cg)^2);
    J_y_back_arm   = m_arm * ((1/12)*(l_b)^2 + (l_b/2-X_cg)^2);
    
    J_y_left_right_motors = 2 * m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (X_cg)^2);
    J_y_left_right_props  = 2 * m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (X_cg)^2);
    J_y_left_arm   = m_arm * ((1/12)*(l_l)^2 + (X_cg)^2);
    J_y_right_arm  = m_arm * ((1/12)*(l_r)^2 + (X_cg)^2);
    
    J_y_body = m_cube * ((1/6)*d_cube^2 + (X_cg)^2);
    
    J_Y = J_y_body + J_y_back_arm + J_y_back_prop + J_y_back_motor + J_y_front_arm + J_y_front_prop + J_y_front_motor + J_y_right_arm + J_y_left_arm + J_y_left_right_props + J_y_left_right_motors;
    
    % Calculating J_x
    J_x_left_motor = m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (l_l-Y_cg)^2);
    J_x_left_prop  = m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (l_l-Y_cg)^2);
    J_x_left_arm   = m_arm * ((1/12)*l_l^2 + (l_l/2-Y_cg)^2);
    
    J_x_right_motor = m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (l_r-Y_cg)^2);
    J_x_right_prop  = m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (l_r-Y_cg)^2);
    J_x_right_arm   = m_arm * ((1/12)*(l_r)^2 + (l_r/2-Y_cg)^2);
    
    J_x_front_back_motors = 2 * m_motor * ((1/4)*r_motor^2 + (1/12)*t_motor^2 + (Y_cg)^2);
    J_x_front_back_props  = 2 * m_prop * ((1/4)*r_prop^2 + (1/12)*t_prop^2 + (Y_cg)^2);
    J_x_front_arm   = m_arm * ((1/12)*(l_f)^2 + (Y_cg)^2);
    J_x_back_arm  = m_arm * ((1/12)*(l_b)^2 + (Y_cg)^2);
    
    J_x_body = m_cube * ((1/6)*d_cube^2 + (Y_cg)^2);
    
    J_X = J_x_body + J_x_back_arm + J_x_front_arm + J_x_front_back_props + J_x_front_back_motors + J_x_right_arm + J_x_right_prop + J_x_right_motor + J_x_left_arm + J_x_left_prop + J_x_left_motor;
    
    % Calculating J_z
    J_z_front_motor = m_motor * ((1/2) * r_motor^2 + d_f_motor_prop^2);
    J_z_front_prop  = m_prop  * ((1/2) * r_prop^2  + d_f_motor_prop^2);
    J_z_front_arm   = m_arm   * ((1/12) * (l_f)^2 + (d_f_arm)^2);
    
    J_z_back_motor = m_motor * ((1/2) * r_motor^2 + d_b_motor_prop^2);
    J_z_back_prop  = m_prop  * ((1/2) * r_prop^2  + d_b_motor_prop^2);
    J_z_back_arm   = m_arm   * ((1/12) * (l_b)^2 + (d_b_arm)^2);
    
    J_z_left_motor = m_motor * ((1/2) * r_motor^2 + d_l_motor_prop^2);
    J_z_left_prop  = m_prop  * ((1/2) * r_prop^2  + d_l_motor_prop^2);
    J_z_left_arm   = m_arm   * ((1/12) * (l_l)^2 + (d_l_arm)^2);
    
    J_z_right_motor = m_motor * ((1/2) * r_motor^2 + d_r_motor_prop^2);
    J_z_right_prop  = m_prop  * ((1/2) * r_prop^2  + d_r_motor_prop^2);
    J_z_right_arm   = m_arm   * ((1/12) * (l_r)^2 + (d_r_arm)^2);
    
    J_z_body = m_cube * ((1/6)*d_cube^2 + (d_body)^2);
    
    J_Z = J_z_body + J_z_right_arm + J_z_right_prop + J_z_right_motor + J_z_left_arm + J_z_left_prop + J_z_left_motor + J_z_back_arm + J_z_back_prop + J_z_back_motor + J_z_front_arm + J_z_front_prop + J_z_front_motor;
    
    % Calculating J_xy
    J_xy_front_motor = m_motor * (l_f - X_cg) * (0 - Y_cg);
    J_xy_front_prop  = m_prop  * (l_f - X_cg) * (0 - Y_cg);
    J_xy_front_arm   = m_arm   * ((l_f / 2) - X_cg) * (0 - Y_cg);
    
    J_xy_back_motor = m_motor * (l_b - X_cg) * (0 - Y_cg);
    J_xy_back_prop  = m_prop  * (l_b - X_cg) * (0 - Y_cg);
    J_xy_back_arm   = m_arm   * ((l_b / 2) - X_cg) * (0 - Y_cg);
    
    J_xy_right_motor = m_motor * (0 - X_cg) * (l_r - Y_cg);
    J_xy_right_prop  = m_prop  * (0 - X_cg) * (l_r - Y_cg);
    J_xy_right_arm   = m_arm   * (0 - X_cg) * ((l_r / 2) - Y_cg);
    
    J_xy_left_motor = m_motor * (0 - X_cg) * (l_l - Y_cg);
    J_xy_left_prop  = m_prop  * (0 - X_cg) * (l_l - Y_cg);
    J_xy_left_arm   = m_arm   * (0 - X_cg) * ((l_l / 2) - Y_cg);
    
    J_xy_body = m_cube * (0 - X_cg) * (0 - Y_cg);
    
    J_XY = J_xy_front_motor + J_xy_front_prop + J_xy_front_arm + ...
           J_xy_back_motor + J_xy_back_prop + J_xy_back_arm + ...
           J_xy_right_motor + J_xy_right_prop + J_xy_right_arm + ...
           J_xy_left_motor + J_xy_left_prop + J_xy_left_arm + ...
           J_xy_body;
    
    % Total J_XZ & J_YZ
    % Because we assume components are in the same horizontal plane (z-plane)
    % Which it causes our cross product of J_xz and J_yz is equal to 0.
    J_XZ = 0;
    J_YZ = 0;
    
    J_matrix_body = [ J_X , -J_XY, -J_XZ;
                       -J_XY ,  J_Y, -J_YZ;
                       -J_XZ , -J_YZ,  J_Z];

    CG_matrix     = [X_cg;
                     Y_cg];

end

function state_dot = calculate_quadcopter_eom(state, control_inputs, J_matrix_body, m_tot, k1, k2, g, l_f, l_r, l_b, l_l)
% Function to calculate the equations of motion of the quadcopter
% Inputs:
%   state: [pn; pe; h; u; v; w; phi; theta; psi; p; q; r]
%   control_inputs: [delta_f; delta_r; delta_b; delta_l]
%   J_matrix_body: Moment of inertia matrix [Jx, Jy, Jz]
%   m_tot: Total mass of the quadcopter
%   k1: Thrust coefficient
%   k2: Torque coefficient
%   g: Gravitational acceleration
%   l_f, l_r, l_b, l_l: Individual arm lengths for front, right, back, and left rotors
% Outputs:
%   state_dot: Time derivatives of the state vector

    % Extract state variables
    pn = state(1); pe = state(2); h = state(3);
    u = state(4); v = state(5); w = state(6);
    phi = state(7); theta = state(8); psi = state(9);
    p = state(10); q = state(11); r = state(12);
    
    % Extract control inputs
    delta_f = control_inputs(1);
    delta_r = control_inputs(2);
    delta_b = control_inputs(3);
    delta_l = control_inputs(4);
    
    % Position Dynamics
    position_matrix = [
        cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
        sin(theta), -sin(phi)*cos(theta), -cos(phi)*cos(theta)
    ];
    position_vector = [u; v; w];
    position_dot = position_matrix * position_vector;
    
    % Linear Velocity Dynamics
    linear_velocity_matrix = [
        0, r, -q;
        -r, 0, p;
        q, -p, 0
    ];
    gravity_vector = [-g*sin(theta); g*cos(theta)*sin(phi); g*cos(theta)*cos(phi)];
    control_force = [0; 0; -k1 * (delta_f + delta_r + delta_b + delta_l) / m_tot];
    linear_velocity_dot = linear_velocity_matrix * [u; v; w] + gravity_vector + control_force;
    
    % Angular Velocity Dynamics
    angular_velocity_matrix = [
        1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi), -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)
    ];
    angular_velocity_dot = angular_velocity_matrix * [p; q; r];
    
    % Rotational Dynamics
    % Torques due to rotor thrusts
    tau_x = l_f * k1 * delta_f - l_b * k1 * delta_b; % Front-back torques
    tau_y = l_r * k1 * delta_r - l_l * k1 * delta_l; % Right-left torques
    tau_z = k2 * (-delta_f + delta_r - delta_b + delta_l); % Yaw torque due to rotor drag
    
    % Total torques
    control_torque = [tau_x; tau_y; tau_z];
    
    % Gyroscopic effects
    rotational_dynamics_matrix = [
        (J_matrix_body(2, 2) - J_matrix_body(3, 3)) / J_matrix_body(1, 1), 0, 0;
        0, (J_matrix_body(3, 3) - J_matrix_body(1, 1)) / J_matrix_body(2, 2), 0;
        0, 0, (J_matrix_body(1, 1) - J_matrix_body(2, 2)) / J_matrix_body(3, 3)
    ];
    rotational_coupling = [q * r; p * r; p * q];
    rotational_velocity_dot = rotational_dynamics_matrix * rotational_coupling + J_matrix_body \ control_torque;
    
    % Combine EOMs
    state_dot = [position_dot; linear_velocity_dot; angular_velocity_dot; rotational_velocity_dot];
end


% Define the parameters
% Arm length
max_arm_length = 0.2;
min_arm_length = 0.4;
number_of_discretization = 3;
arm_length_num = linspace(min_arm_length, max_arm_length, number_of_discretization);

% Initialize combination group
combination_length = [];

% Creating all possible combinations by different length
for i = 1:length(arm_length_num)
    l_f_abs = arm_length_num(i);
    for j = 1:length(arm_length_num)
        l_r_abs = arm_length_num(j);
        for k = 1:length(arm_length_num)
            l_b_abs = arm_length_num(k);
            for z = 1:length(arm_length_num)
                l_l_abs = arm_length_num(z); 
                % Append the current combination with correct signs
                combination_length = [combination_length; ...
                    l_f_abs, l_r_abs, -l_b_abs, -l_l_abs];
            end
        end
    end
end

% Fixed(hard) property parameters
t_motor = 31/1000;   % Motor thickness (m)
t_prop = 2/1000;    % Propeller thickness (m)
d_cube = 0.10;    % Cube dimension (m)
m_motor = 58/1000;   % Motor mass (kg)
m_prop = 21/1000;    % Propeller mass (kg)
m_arm = 65/1000;     % Arm mass (kg)
m_cube = 0.69;    % Cube mass (kg)
r_motor = 28/1000;   % Motor radius (m)
r_prop = 25.4/100;    % Propeller radius (m)

g = 9.81; % Gravity (kg/m^2)
k1 = 1;   % Thrust coefficient
k2 = 0.2; % Torque coefficient

% % Define the throttle percentages and corresponding thrusts
% throttle_table = [50, 62.5, 75, 88, 100];  % Throttle percentages (%)
% thrust_table = [666, 832.5, 999, 1165.5, 1332];  % Single thrust (g)

initial_condition_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% Specify the folder name
output_folder = 'Modelling_AB_results';

% Create the folder in the current working directory
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

for c = 1:size(combination_length, 1)  % Use size for correct row count
    l_f = combination_length(c, 1);    % Front arm length
    l_r = combination_length(c, 2);    % Right arm length
    l_b = combination_length(c, 3);    % Back arm length
    l_l = combination_length(c, 4);    % Left arm length
    config = [l_f, l_r, l_b, l_l];

    % Define state
    control_matrix_x = [pn; pe; h; u; v; w; phi; theta; psi; p; q; r];
    control_matrix_u = [delta_f; delta_r; delta_b; delta_l]; 
    
    % Calculate CG matrix, J and m_tot
    [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(l_f, l_r, l_b, l_l, t_motor, t_prop, d_cube, m_motor, m_prop, m_arm, m_cube, r_motor, r_prop);
    
    % Calculate equation of motion
    equation_of_motion = calculate_quadcopter_eom(control_matrix_x, control_matrix_u, J_matrix_body, m_tot, k1, k2, g, l_f, l_r, l_b, l_l);
    
    % Jacobian Matrix:
    jaco_about_x = jacobian(equation_of_motion, control_matrix_x);
    jaco_about_u = jacobian(equation_of_motion, control_matrix_u);
    
    % Initial Conditions:
    syms psi_zero pn_zero pe_zero h_zero
    A_matrix = subs(jaco_about_x, [pn pe h u v w phi theta psi p q r], initial_condition_state);
    B_matrix = subs(jaco_about_u, [delta_f delta_r delta_b delta_l], [(m_tot * g / (4 * k1)), (m_tot * g / (4 * k1)), (m_tot * g / (4 * k1)), (m_tot * g / (4 * k1))]);
    
    A = double(A_matrix);
    B = double(B_matrix);

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

