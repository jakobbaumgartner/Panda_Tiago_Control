pos = [0 0.5 0 -0.5 1 1.5 0];
% pos = [0 0 0 -0.5 2 1.5 1];
% 
% pos = zeros(7,1)
% pos(1) = 1/
% pos(2) = 1


arm_joints_publisher = rospublisher("/arm_controller/command")


arm_msg = rosmessage("trajectory_msgs/JointTrajectory");
joints_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');

arm_msg.JointNames = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'};

joints_msg.Positions = pos;


joints_msg.TimeFromStart = rosduration(4);

arm_msg.Points(1) = joints_msg;

arm_joints_publisher.send(arm_msg)

% directKinematics(pos(1), pos(2), pos(3), pos(4), pos(5), pos(6), pos(7))