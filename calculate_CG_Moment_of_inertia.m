%% Calculate CG location and mass moment of inertia of the quadcopter J_matrix_body
function [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(config)
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

l_f = config(1);
l_r = config(2);
l_b = config(3);
l_l = config(4);

% Assumption:
% 1. CG in z-axis is not changing;
% 2. Spininig props as a disk;
% 3. The disk has no thickness;
% 4. All the length l_b, l_f, l_l, l_r are all treat as signed
% +y = right rotor
% +x = front rotor
% +z = point down
t_motor = 31/1000;   % Motor thickness (m)
t_prop = 2/1000;    % Propeller thickness (m)
d_cube = 0.10;    % Cube dimension (m)
m_motor = 100/1000;   % Motor mass (kg)
m_prop = 21/1000;    % Propeller mass (kg)
m_arm = 65/1000;     % Arm mass (kg)
m_cube = 1.74;    % Cube mass (kg)
r_motor = 28/1000;   % Motor radius (m)
r_prop = 25.4/100;    % Propeller radius (m)

g = 9.81; % Gravity (kg/m^2)
k1 = 1;   % Thrust coefficient
k2 = 0.2; % Torque coefficient

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

