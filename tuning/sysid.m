function sysid()

% load data
data_file = './turtlebot_dyn_logging_10_27_2025.txt';
data = readmatrix(data_file);
t = data(:,1);
x = data(:,2);
y = data(:,3);
v = data(:,4);
th = data(:,5);
om = data(:,6);
x_r = data(:,7);
y_r = data(:,8);
v_r = data(:,9);
th_r = data(:,10);
om_r = data(:,11);
v_cmd = data(:,12);
th_cmd = data(:,13);
om_cmd = data(:,14);

t = t - t(1);


% position plot
figure

subplot(2,1,1);
hold on;
plot(t, x_r, '-g')
plot(t, x, '-k')
title('x')
ylabel('[m]')

subplot(2,1,2);
hold on;
plot(t, y_r, '-g')
plot(t, y, '-k')
title('y')
ylabel('[m]')

xlabel('[sec]')

% velocities plot
figure

subplot(2,1,1);
hold on;
plot(t, v_r, '-g')
plot(t, v_cmd, '--r')
plot(t, v, '-k')
ylabel('[m/s]')
title('v')

subplot(2,1,2);
hold on;
plot(t, om_r*(180/pi), '-g')
plot(t, om_cmd*(180/pi), '--r')
plot(t, om*(180/pi), '-k')
ylabel('[deg/s]')
title('\omega')


%% sys id

% time step of data
dt = mean(diff(t));

% get selected portion of trajectory
t_min = 64;
t_max = 87;
ind_min = find(t >= t_min, 1);
ind_max = find(t >= t_max, 1);

t_sysid = t(ind_min:ind_max);
v_sysid = v(ind_min:ind_max);
v_cmd_sysid = v_cmd(ind_min:ind_max);
om_sysid = om(ind_min:ind_max);
om_cmd_sysid = om_cmd(ind_min:ind_max);

% velocity: minimize sum of squared errors
tau0 = 0.05;
tau_v = fminunc(@sysid_cost_v, tau0);
fprintf('tau_v = %f\n', tau_v)

% velocity: validation plot
v_model = get_model_traj(v_cmd_sysid, tau_v, dt, v_sysid(1));
figure
hold on;
plot(t_sysid, v_cmd_sysid, '--r')
plot(t_sysid, v_model, '-b')
plot(t_sysid, v_sysid, '-k')
xlabel('[sec]')
ylabel('[m/s]')

% angular velocity: minimize sum of squared errors
tau0 = 0.05;
tau_om = fminunc(@sysid_cost_om, tau0);
fprintf('tau_om = %f\n', tau_om)

% angular velocity: validation plot
om_model = get_model_traj(om_cmd_sysid, tau_om, dt, om_sysid(1));
figure
hold on;
plot(t_sysid, om_cmd_sysid*(180/pi), '--r')
plot(t_sysid, om_model*(180/pi), '-b')
plot(t_sysid, om_sysid*(180/pi), '-k')
xlabel('[sec]')
ylabel('[deg/s]')






% cost functions
    function J = sysid_cost_v(tau)
        v_model = get_model_traj(v_cmd_sysid, tau, dt, v_sysid(1));
        J = sum((v_model-v_sysid).^2)/length(v_model);
    end

    function J = sysid_cost_om(tau)
        om_model = get_model_traj(om_cmd_sysid, tau, dt, om_sysid(1));
        J = sum((om_model-om_sysid).^2)/length(om_model);
    end

    function model = get_model_traj(cmd, tau, dt, x0)
        model = zeros(size(cmd));
        model(1) = x0;
        for ii = 2:length(model)
            model(ii) = model(ii-1) + (dt/tau)*(cmd(ii-1)-model(ii-1));
        end
    end


end