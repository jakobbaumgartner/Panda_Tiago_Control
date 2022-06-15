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

goal = goal_02;


% get GAZEBO positions

sub_gazebolinks = rossubscriber('/gazebo/link_states');
gazebo_positions = receive(sub_gazebolinks);
position_basefootprint = gazebo_positions.Pose(2).Position;
orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W]);

pos_x = position_basefootprint.Y;
pos_y = position_basefootprint.X;
pos_fi = 999;

% while 1
%     gazebo_positions = receive(sub_gazebolinks);
%     orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
%     eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W])
%     rad2deg(eulZYX)
%     pause(0.1)
% end


% control loop - run until EE position is close
max_dist = 0.05; % max distnace from end point
p_vel = [0.1 0.01]'; % speed p constant


while norm([position_basefootprint.X, position_basefootprint.Y] - goal) > max_dist

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

    dist = [pos_x, pos_y] - goal 
    diff_val = [ 0 0 ];
    diff_val(1) = norm(dist);


    diff_val(2) = atan(pos_x/pos_y)%+ eulZYX(3) 
    pos_fi = atan(pos_x/pos_y);
    rad2deg(diff_val(2))

%     fix for 3 in 4 kvadrant & abs fi

    % kvadrant 1
    if pos_y > 0 && pos_x < 0
        pos_fi = 2*pi + diff_val(2);
    end

    % kvadrant 4
    if pos_y < 0 && pos_x > 0
        diff_val(2) = pi + diff_val(2);
        pos_fi = diff_val(2);
    end

    % kvadrant 3
    if pos_y < 0 && pos_x < 0
            diff_val(2) = -pi + diff_val(2);
            pos_fi = 2*pi + diff_val(2);
    end

    disp("Rotation:  " + string(rad2deg(diff_val(2) )))
    disp("Rotation fi-abs:  " + string(rad2deg(pos_fi)))
    disp("X:  " + string(pos_x) + "  Y:  " + string(pos_y))
        
    vel = diff_val .* p_vel;


    msg_diff.Linear.X = 0; %vel(1);
    msg_diff.Angular.Z = diff_val(2) * 0.1;

    diff_w_data = [diff_w_data(1,2:end) rad2deg(diff_val(2))];
    diff_fi_data = [diff_fi_data(1,2:end) eulZYX(3)];
    x_data = [x_data(1,2:end) dist(1)];
    y_data = [y_data(1,2:end) dist(2)];

%     hold on
%     plot(diff_fi_data)


    
%     pub_diff.send(msg_diff)
    

    gazebo_positions = receive(sub_gazebolinks);
    pause(0.01)
    disp(string(randi(100)))
    position_basefootprint = gazebo_positions.Pose(2).Position;
    orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
    eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W])

end





end