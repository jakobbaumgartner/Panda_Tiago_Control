function T = geometric_panda_pmb2_joints (q)

    fi1b = q(1);
    d2b = q(2);
    fi1 = q(3);
    fi2 = q(4);
    fi3 = q(5);
    fi4 = q(6);
    fi5 = q(7);
    fi6 = q(8);
    fi7 = q(9);

    % calculates direct geometric model of panda arm with added base, base
    % considered as rotational + linear joints
    % --------------------------------------------------------------
    % returns transformation matrix of EE, based on joint values 

    % joint b1 - rotacija
    a1b = 0;
    d1b = 0.83;
    alpha1b = pi/2;

    A0b1b = getTransformationA(a1b, d1b, alpha1b, fi1b)
    
    % joint b2 - translacija
    a2b = 0;
    d2b0 = 0;
    alpha2b = -pi/2;
    fi2b0 = 0;

    A1b2b = getTransformationA(a2b, d2b0 + d2b, alpha2b, fi2b0)

    % joint 1
    a1 = 0;
    d1 = 0.333;
    alpha1 = -pi/2;

    A01 = getTransformationA(a1, d1, alpha1, fi1);
    
    % joint 2
    a2 = 0;
    d2 = 0;
    alpha2 = pi/2;

    A12 = getTransformationA(a2, d2, alpha2, fi2);
    
    % joint 3
    a3 = 0.0825;
    d3 = 0.316;
    alpha3 = pi/2;

    A23 = getTransformationA(a3, d3, alpha3, fi3);

    % joint 4
    a4 = 0.0825;
    d4 = 0;
    alpha4 = pi/2;
    fi40 = pi;

    A34 = getTransformationA(a4, d4, alpha4, fi40 + fi4);

    % joint 5
    a5 = 0;
    d5 = 0.384;
    alpha5 = pi/2;
    fi50 = pi;

    A45 = getTransformationA(a5, d5, alpha5, fi50 + fi5);

    % joint 6
    a6 = -0.0880;
    d6 = 0;
    alpha6 = pi/2;
    fi60 = pi;

    A56 = getTransformationA(a6, d6, alpha6, fi60 + fi6);

    % joint 7
    a7 = 0;
    d7 = -0.1070;
    alpha7 = 0;

    A67 = getTransformationA(a7, d7, alpha7, fi7);


    % assemble transformation matrix

    T = A0b1b * A1b2b * A01 * A12 * A23 * A34 * A45 * A56 * A67;
 
end