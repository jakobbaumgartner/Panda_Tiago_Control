function A = getTransformationA(a,d,alpha,fi)

% getTransformationA - get one step of direct geometric model
% --------------------------------------------------------------
% creates transformation between two Coordinate systems, 
% based on DH parameters

A = [ cos(fi) -sin(fi)*cos(alpha) sin(fi)*sin(alpha) a*cos(fi) ;
      sin(fi) cos(fi)*cos(alpha) -cos(fi)*sin(alpha) a*sin(fi) ;
      0       sin(alpha)          cos(alpha)         d ;
      0       0                   0                  1];


% A = [ cos(fi) 0 sin(fi) 0 ;
%       sin(fi) 0 -cos(fi) 0 ;
%       0 1 0 d ;
%       0 0 0 1] ;

end