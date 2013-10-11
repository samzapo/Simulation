function Kinematics

    %% Problem 1
    function qdd = calc_QDD(Xdd,Xddsw)
        thdd = Xdd(3);
        Rdd = [-cos(th) sin(th) ;-sin(th) -cos(th)]*Xddsw(3);
        qdd = [Rdd*[Xdd(1);Xdd(2)] + Xddsw(1:2) ; thdd + Xddsw(3)];
    end
    
    function qd = calc_QD(Xd,Xdsw)
        thd = Xd(3);
        Rd = [-sin(th) -cos(th) ;cos(th) -sin(th)]*Xdsw(3);
        qd = [Rd*[Xd(1);Xd(2)] + Xdsw(1:2) ; thd + Xdsw(3)];
    end

    function q = calc_Q(X,Xsw)
        th = X(3);
        R = [cos(th) -sin(th) ;sin(th) cos(th)];
        q = [R*[X(1);X(2)] + Xsw(1:2) ; th + Xsw(3)];
    end
    %% Problem 2
    
    
    %% Problem 3
    Xdsw = [;1]
    Xsw = [1;-1;pi/4]
    
    Xd = [;-2]
    X = [5;5;pi/6]

    %% Problem 4
    
    
    %% Problem 5
end

