% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
   X1 = v2t(x1);
   X2 = v2t(x2);
   Z = v2t(z);
   e = t2v(Z\(X1\X2));

   t_i = x1(1:2,1);
   t_j = x2(1:2,1);
   t_ij = z(1:2,1);
   R_i = [cos(x1(3)) -sin(x1(3)); sin(x1(3)) cos(x1(3))];
   R_ij = [cos(z(3)) -sin(z(3)); sin(z(3)) cos(z(3))];
   R_i_diff = [-sin(x1(3)) -cos(x1(3)); cos(x1(3)) -sin(x1(3))]; %diff w-r.t to theta of x1
   
   A = [-R_ij'*R_i',R_ij'*R_i_diff'*(t_j-t_i);0,0,-1];
   B = [R_ij'*R_i',[0;0] ;0,0,1];
end;
