function status = base_of_simple_velocity_control_geometric_wheels()


global pub_diff
global msg_diff


pub_diff = rospublisher('/mobile_base_controller/cmd_vel')
msg_diff = rosmessage(pub_diff)

diff_w_data = zeros(1,100);
diff_fi_data = zeros(1,100);
x_data = zeros(1,100);
y_data = zeros(1,100);




% goal position

goal_base_1 = [0.9224, -2.8224];  
goal_base_2 = [5.4355, 4.0976];
goal_base_center = [0 0];
goal_02 = [0 2];

pos_goal = goal_base_1;


% get GAZEBO positions

sub_gazebolinks = rossubscriber('/gazebo/link_states');
gazebo_positions = receive(sub_gazebolinks);
position_basefootprint = gazebo_positions.Pose(2).Position;
orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W]);

pos_x = - position_basefootprint.Y;
pos_y = position_basefootprint.X;
pos_fi = 999;

% control loop - run until EE position is close
max_dist = 0.05; % max distnace from end point
p_vel = [0.1 0.01]'; % speed p constant


while norm([position_basefootprint.X, position_basefootprint.Y] - pos_goal) > max_dist

    % baza wheels

    r = 0.0985; % wheel radius
    L = 0.4044; % wheel seperation
    z_base = 0.2976; % 0.83;

    Kbase = zeros(2,2);

    Kbase (1,:)= [r/2 r/2];
    Kbase(2,:) = [-r/L r/L];    

    pinv_JB = pinv(Kbase);
    
    pos_x = - position_basefootprint.Y;
    pos_y = position_basefootprint.X;

    dist = [pos_x, pos_y] - pos_goal 
    diff_val = [ 0 0 ];
    diff_val(1) = norm(dist);


%     [diff_val(2), pos_fi] = gazebo_calc_base_orientation(pos_x,pos_y)
    [diff_val(2), pos_fi, fi_error] = calc_base_target_orientation(pos_x, pos_y, pos_goal(1), pos_goal(2), eulZYX(3))

%     disp("Psi relat: " + string(psi_relat))
    disp("Rotation locat:  " + string(rad2deg(diff_val(2))))
    disp("Rotation base rotac  " + string(rad2deg(eulZYX(3))))
    disp("Rotation error  " + string(rad2deg(fi_error)))

    disp("X:  " + string(pos_x) + "  Y:  " + string(pos_y))
%     disp("Angle correction:  " + string(psi_relat + rad2deg(diff_val(2))))
         
    vel = diff_val .* p_vel;


    msg_diff.Linear.X = vel(1);
    msg_diff.Angular.Z = fi_error;


%     hold on
%     plot(diff_fi_data)

    pub_diff.send(msg_diff)
    
    

    % read new position data 
    gazebo_positions = receive(sub_gazebolinks);
    pause(0.01)
    position_basefootprint = gazebo_positions.Pose(2).Position;
    orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
    eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W])

end





end