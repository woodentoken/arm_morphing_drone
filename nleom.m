function [outputArg1,outputArg2] = nleom(inputArg1,inputArg2)
    %NLEOM Summary of this function goes here
    %   Detailed explanation goes here


    % Calculate CG matrix, J and m_tot
    [CG_matrix, J_matrix_body, m_tot] = calculate_CG_Moment_of_inertia(l_f, l_r, l_b, l_l, t_motor, t_prop, d_cube, m_motor, m_prop, m_arm, m_cube, r_motor, r_prop);
    % Calculate equation of motion
    equation_of_motion = calculate_quadcopter_eom(control_matrix_x, control_matrix_u, J_matrix_body, m_tot, k1, k2, g, l_f, l_r, l_b, l_l);

    equation_of_motion
end