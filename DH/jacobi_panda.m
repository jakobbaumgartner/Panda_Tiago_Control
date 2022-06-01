function J = jacobi_panda (fi1, fi2, fi3, fi4, fi5, fi6, fi7)

    % calculates direct geometric model of panda arm
    % --------------------------------------------------------------
    % returns transformation matrix of EE, based on joint values 

    
    % link: https://www.rosroboticslearning.com/jacobian


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


    % transform matrix
    T7 = A01 * A12 * A23 * A34 * A45 * A56 * A67;
    
    p7 = T7(1:3,4)

    z0 = [0 0 1]';


    % initialize Jacobi matrix
    J = zeros(6,7);
    
    % first column

    p = [0 0 0]';

    z = z0;

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,1) = jp;
    J(4:6,1) = jo;


    % second column

    z = A01(1:3,1:3)*z0;

    p = A01(1:3,4)

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,2) = jp;
    J(4:6,2) = jo;


    % third column

    z = A01(1:3,1:3)*A12(1:3,1:3)*z0;

    T = A01*A12;

    p = T(1:3,4);

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,3) = jp;
    J(4:6,3) = jo;

    
    % fourth column

    z = A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*z0;

    T = A01*A12*A23;

    p = T(1:3,4);

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,4) = jp;
    J(4:6,4) = jo;


    % fifth column

    z = A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*z0;

    T = A01*A12*A23*A34;

    p = T(1:3,4);

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,5) = jp;
    J(4:6,5) = jo;


    % sixth column

    z = A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*z0;

    T = A01*A12*A23*A34*A45;

    p = T(1:3,4);

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,6) = jp;
    J(4:6,6) = jo;


    % seventh column

    z = A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A34(1:3,1:3)*A45(1:3,1:3)*A56(1:3,1:3)*z0;

    T = A01*A12*A23*A34*A45*A56;

    p = T(1:3,4);

    jp = cross(z,(p7-p));
    jo = z;

    J(1:3,7) = jp;
    J(4:6,7) = jo;
    

 
end