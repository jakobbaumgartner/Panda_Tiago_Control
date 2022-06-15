function status = simple_velocity_control_geometric_wheels()

global pub_diff
global msg_diff

pub_diff = rospublisher('/mobile_base_controller/cmd_vel')
msg_diff = rosmessage(pub_diff)


% velocity control, for testing jacobis in simulation  

% Ce je casovno trajanje spremembe izredno kratko, potem lahko na
% situacijo gledamo kot na spremembo hitrosti v poljubnem sklepu ( dqj),
% ki se odraza v hitrosti vrha robota v obliki translacijske hitrosti dp ali
% rotacijske hitrosti ω izrazena glede na osi baznega koordinatnega sis-
% tema. Omenjeno relacijo med hitrostmi v sklepih dq in hitrostmi vrha
% robota dp (v) opisuje Jacobijeva matrika, ki jo oznacujemo s crko J in
% zapisemo z enacbo:
% v = dp = J(q) dq

% goal position

% goal_base = [0.9224, -2.8224, -0.0985];  

goal_EE_t = [5.4355, 4.0976, 1.1965];
% goal_EE_t = [0 0 0 ];

% goal_EE_q = [-0.7296, 0.4622, -0.4672, -0.1892];

% get GAZEBO positions

sub_gazebolinks = rossubscriber('/gazebo/link_states');
gazebo_positions = receive(sub_gazebolinks);
position_pandalink7 = gazebo_positions.Pose(17).Position
position_basefootprint = gazebo_positions.Pose(2).Position;

% arm joint positions
arm_state = rostopic("echo", '/arm_controller/state');
joint_states = arm_state.Actual.Positions;

% control loop - run until EE position is close
max_dist = 1; % max distnace from end point
p_vel = 0.01; % speed p constant

while norm([position_pandalink7.X, position_pandalink7.Y, position_pandalink7.Z] - goal_EE_t) > max_dist
    
    J = jacobi_panda_pmb2_wheels (joint_states); % jacobi
    pinv_J = pinv(J); % pseudo-inverse

    % baza wheels

    r = 0.0985; % wheel radius
    L = 0.4044; % wheel seperation
    z_base = 0.2976; % 0.83;

    Kbase = zeros(2,2);

    Kbase (1,:)= [r/2 r/2];
    Kbase(2,:) = [r/L -r/L];    

    pinv_JB = pinv(Kbase);
    
    dist = [[position_pandalink7.X, position_pandalink7.Y, position_pandalink7.Z] - goal_EE_t 0 0 0]     %% PRBOLEM PROBABLY HERE
    dist(3) = 0;
    diff_val = [ 0 0 ];
    diff_val(1) = norm(dist);

    diff_val(2) = atan(dist(1)/dist(2))


    
    vel = p_vel * diff_val;
%     q_vel = pinv_J * vel';
    q_vel = pinv_JB * vel(1:2)';


    % send diff drive commands
    diff_controller(q_vel(1:2));

    % send arm commands
%     arm_controller(q_vel(3:9));
       
    % arm joint positions
%     arm_state = rostopic("echo", '/arm_controller/state');
%     joint_states = arm_state.Actual.Positions;

    gazebo_positions = receive(sub_gazebolinks);
    position_pandalink7 = gazebo_positions.Pose(17).Position
    position_basefootprint = gazebo_positions.Pose(2).Position;

    pause(0.1)

end





end