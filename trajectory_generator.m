waypoints_position = [
    0 0 0;
    10 0 0;
    10 10 0;
];
num_waypoints = size(waypoints_position);

waypoints_velocity = [
    0 0 0;
    5 0 0; % reach 5 m/s of speed;
    0 0 0;
];

accs = [
    0 0 0;
    1 0 0; % accelerate from rest
    0 0 0;
];
jerks = zeros(num_waypoints);
snaps = zeros(num_waypoints);
yaws = [
    0;
    0;
    90; % end at a 90 degree right turn
    ];

time_of_arrivals = [
    0;
    5; % 5 seconds to accelerate;
    6; % 5 seconds to arrest acceleration and turn 90 degrees;
];

T = multirotorFlightTrajectory(waypoints_position, waypoints_velocity, accs, jerks, snaps, yaws, time_of_arrivals);
ax = show(T, NumSamples=100);
title('sample trajectory (right turn)')
axis equal;
grid on;