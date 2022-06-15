function [diff_val_2, pos_fi] = gazebo_calc_base_orientation(pos_x,pos_y)

    diff_val_2 = atan(pos_x/pos_y);%+ eulZYX(3) 
    pos_fi = atan(pos_x/pos_y);
    rad2deg(diff_val_2)

%     fix for 3 in 4 kvadrant & abs fi

    % kvadrant 1
    if pos_y > 0 && pos_x < 0
        pos_fi = 2*pi + diff_val_2;
    end

    % kvadrant 4
    if pos_y < 0 && pos_x > 0
        diff_val_2 = pi + diff_val_2;
        pos_fi = diff_val_2;
    end

    % kvadrant 3
    if pos_y < 0 && pos_x < 0
            diff_val_2 = - pi + diff_val_2;
            pos_fi = 2*pi + diff_val_2;
    end

end