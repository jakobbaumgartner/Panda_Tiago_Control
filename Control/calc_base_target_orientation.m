function [pos_fi, rot_error] = calc_base_target_orientation(pos_base_x, pos_base_y, pos_goal_x, pos_goal_y, rot_base)

pos_x = pos_goal_x - pos_base_x;
pos_y = pos_goal_y - pos_base_y;

rot_error = 999;

diff_val_2 = atan(pos_y/pos_x );%+ eulZYX(3) 
pos_fi = ((atan(pos_x/pos_y)));
rot_base = ((rot_base));

if pos_x >= 0 && pos_y >= 0
   
    rot_error = rot_base + pos_fi;

    if rot_error > pi
        rot_error = - 2*pi + rot_error;
    end

    rot_error = - rot_error;

end

if pos_x < 0 && pos_y >0

        rot_error = 2*pi - rot_base - pos_fi;

        if rot_error > pi
                 rot_error = - 2*pi + rot_error;
        end


end

if pos_x > 0 && pos_y < 0

    rot_error =  pi +  pos_fi + rot_base;

    if rot_error > pi

        rot_error = -(2*pi - rot_error);

    end
    
    rot_error = - rot_error;


end


if pos_x < 0 && pos_y < 0

    rot_error = pos_fi   + pi/2 -  rot_base;

    if rot_error > pi

        rot_error = -(2*pi - rot_error);

    end
    
end

end