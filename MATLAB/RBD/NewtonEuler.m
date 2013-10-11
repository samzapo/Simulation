function [F,t] = NewtonEuler(c,m,a,v,Icm)
    function sksym = skew(vec)
        sksym = [ 0 -vec(3) vec(2); vec(3) 0 -vec(1); -vec(2) vec(1) 0];
    end
    acm = a(1:3);
    aa  = a(4:6);
    vcm = v(1:3);
    w  = v(4:6);
    % F     = total force acting on the center of mass
    % m     = mass of the body
    % acm   = acceleration of the center of mass
    % vcm   = velocity of the center of mass
    % t     = total torque acting about the center of mass
    % Icm   = moment of inertia about the center of mass
    % w     = angular velocity of the body
    % aa    = angular acceleration of the body
    sf = [m*eye(3), -m*skew(c);
          m*skew(c), Icm-m*skew(c)*skew(c)]*[acm;aa] + [m*cross(w,cross(w,c));
                                                        cross(w,(Icm - m*skew(c)*skew(c))*w)];
    F = sf(1:3);
    t = sf(4:6);
end

