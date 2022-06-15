function [v, w] = diff_controller(q_vel)

global pub_diff
global msg_diff

w_left = q_vel(1);
w_right = q_vel(2);
% diff_controller
% inputs: speed left wheel, speed right wheel
% sends command to topic: /mobile_base_controller/cmd_vel 

L = 0.4044; % wheel seperation


v = (w_left + w_right) / 2 * 1;
w = (w_right - w_left) / L * 10;



if abs(v) > 0.5
    v = sign(v) * 0.5;
end
% 
% if  abs(v) < 0.1 && abs(v) > 0.001
%     v = sign(v) * 0.1;
% end
% 
if abs(w) > 1

   w = sign(v) * 0.5;

end

% if abs(w) > 0.1
%     w = sign(w) * 0.1;
% end


msg_diff.Linear.X = v;
msg_diff.Angular.Z = w;

pub_diff.send(msg_diff)

disp("v: " + string(v) + "  w: " + string(w))
disp("w_left: " + string(w_left) + "   w_right: " + string(w_right))





end