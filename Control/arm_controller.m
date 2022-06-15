function arm_controller(q_vel)
% arm_controller
% inputs: speeds of joints q 1-7
% sends command to topic: /arm_controller/command

arm_joints_publisher = rospublisher("/arm_controller/command")

arm_msg = rosmessage("trajectory_msgs/JointTrajectory");
joints_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');

arm_msg.JointNames = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'};

joints_msg.Velocities = q_vel;
joints_msg.Positions = zeros(7,1);
joints_msg.Accelerations = zeros(7,1);
joints_msg.TimeFromStart = rosduration(0.1);

arm_msg.Points(1) = joints_msg;

arm_joints_publisher.send(arm_msg)



end