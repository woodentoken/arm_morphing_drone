%% Equation of motion calculation 
function state_dot = calculate_quadcopter_eom(state, control_inputs, J_matrix_body, m_tot, config)
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

    l_f = config(1);
    l_r = config(2);
    l_b = config(3);
    l_l = config(4);
    load_constant_values

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
    tau_x = l_f * k1 * delta_f + l_b * k1 * delta_b; % Front-back torques
    tau_y = l_r * k1 * delta_r + l_l * k1 * delta_l; % Right-left torques
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
    rotational_velocity_dot = rotational_dynamics_matrix * rotational_coupling + (J_matrix_body \ control_torque);
    
    % Combine EOMs
    state_dot = [position_dot; linear_velocity_dot; angular_velocity_dot; rotational_velocity_dot];
end
