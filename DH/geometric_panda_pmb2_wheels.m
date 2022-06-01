function T = geometric_panda_pmb2_wheels (dT, wl, wr, fi1, fi2, fi3, fi4, fi5, fi6, fi7)

    % calculates direct geometric model of panda arm with added base, base
    % considered as a differential drive
    % --------------------------------------------------------------
    % returns transformation matrix of EE, based on joint values 

    %%% NOT SURE IF ALL THIS WORKS !!!! TESTS, ASK LEON
  
    % base wheels

    r = 0.0985; % wheel radius
    L = 0.4044; % wheel seperation
    z = 0.83;

    s = [r/2 r/2 ; 0 0 ; -r/L, r/L]*[wl wr]';

    dx = s(1);
    dy = s(2);
    dfi = s(3);
    
    xb = dx * dT;
    yb = dy * dT;
    fib = dfi * dT;

    A0b0 = [cos(fib) -sin(fib) 0 xb;
            sin(fib) cos(fib) 0 yb;
            0 0 1 z;
            0 0 0 1]

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

    T = A0b0 * A01 * A12 * A23 * A34 * A45 * A56 * A67;
 
end