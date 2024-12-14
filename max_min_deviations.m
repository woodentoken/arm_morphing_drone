    output = sim("linear_system.slx");
    time = output.tout;

    %% linear response
    linear_simulation_states = squeeze(output.yout{1}.Values.Data);
    linear_simulation_inputs = squeeze(output.yout{2}.Values.Data);
    normalized_error_linear = squeeze(output.yout{5}.Values.Data);

    figure(5000)
    max_pos_dev = max(max(abs(linear_simulation_states(:,1:2))));
    max_att_dev = max(max(abs(linear_simulation_states(:,8:9))));

    xlabel('maximal attitude deviation');
    ylabel('maximal position deviation');

    if K == K_ref.K
        K_ref_flag = 1;
    else
        K_ref_flag = 0;
    end

    entry = [max_pos_dev, max_att_dev, c, K_ref_flag];

    entries = [entries; entry];