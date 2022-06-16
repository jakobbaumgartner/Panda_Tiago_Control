function status = base_of_simple_velocity_control_geometric_wheels(x,y)

global pub_diff
global msg_diff

pub_diff = rospublisher('/mobile_base_controller/cmd_vel')
msg_diff = rosmessage(pub_diff)

% goal position

goal_base_1 = [0.9224, -2.8224];  
goal_base_2 = [5.4355, 4.0976];
goal_base_center = [0 0];
goal_02 = [0 2];

pos_goal = [x,y];


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
max_dist = 1; % max distnace from end point
p_vel = [1 1]'; % speed p constant


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

    dist = [pos_x, pos_y] - pos_goal;
    diff_val = [ 0 0 ];
    diff_val(1) = norm(dist);

    [pos_fi, diff_val(2)] = calc_base_target_orientation(pos_x, pos_y, pos_goal(1), pos_goal(2), eulZYX(3));
% 
%     disp("Rotation base rotac  " + string(rad2deg(eulZYX(3))))
%     disp("Rotation error  " + string(rad2deg(diff_val(2))))
    disp("Base position - > X:  " + string(pos_x) + "  Y:  " + string(pos_y))
         
    vel = diff_val.*p_vel';

    % linear speed limit
    if vel(1) > 0.8
        vel(1) = 0.8;
    end

    % send control message
    msg_diff.Linear.X = vel(1);
    msg_diff.Angular.Z = vel(2);
    pub_diff.send(msg_diff)
    
    

    % read new position data 
    gazebo_positions = receive(sub_gazebolinks);
    pause(0.01)
    position_basefootprint = gazebo_positions.Pose(2).Position;
    orientation_basefootprint = gazebo_positions.Pose(2).Orientation;
    eulZYX = quat2eul([orientation_basefootprint.X orientation_basefootprint.Y orientation_basefootprint.Z orientation_basefootprint.W]);

end

end