function NumericalIntegration
hold on
    %% Problem 1
%{
    function [xdot] = df(x)
        xdot = -20*x;
    end   

    function [X,T] = Euler(df,x0,dt,max)
        X = x0; 
        T = 0;
        i = 1;
        while T(i) < max
            X = [X; X(i) + dt*df(X(i))];
            T = [T;T(i)+dt];
            i = i+1;
        end
    end

    dt = 0.01;
    maxt = 1; 
    x0 = 1;
    [X,T] = Euler(@df,x0,dt,maxt);
    n = size(T,1);
    plot(T,X,'xr')
    hold on;
    X0 = exp(-20*T);
    plot(T,X0,'b')
    
    err = X - X0;
    alpha = err'*err;
   
    axis fill;
    
    fprintf('The numerical integration was able to approximate the actual function\n with a dt <= %f, and an average error of %f over [0..100]\n',dt,(alpha/n));
    %}
    %% Problem 2
    % Problem 2: Given the ODE system [x ? v ?]T = f(x,v) 
    % (defined below), derive ?f and ?f . De- ?x ?v 
    % describe how you would compute this numerically.
    % [xdot ; vdot] = [v ; -x]
    
    %% Problem 3
    %
    % 
    %{
    g = 9.8 ;
    L = 2 ;
    function [theta,omega] = pendulumEuler_semiimplicit(state,dt) 
       theta0 = state(1);
       omega0 = state(2); 
       omega = omega0 + dt*(-g/L*sin(theta0));
       theta = theta0 + dt*omega;
    end

    function [theta,omega] = pendulumEuler_explicit(state,dt) 
       theta0 = state(1);
       omega0 = state(2); 
       omega = omega0 + dt*(-g/L*sin(theta0));
       theta = theta0 + dt*omega0;
    end

    hold on;

    format long;
    
    istate = [pi/2,0];
    % explicit
    maxt = 50;
    dt = 0.001;
    T = 0:dt:maxt+dt;
    trueState = [istate(1) * cos(sqrt(g/L)*T);istate(1) * -sqrt(g/L) * sin(sqrt(g/L)*T)]';

    state = istate;
    for t = 0:dt:maxt
       [theta,omega] = pendulumEuler_explicit(state(end,:),dt);
       state = [state;theta,omega];
    end
    plot(T,state(:,1),'b')
    
    size(state)
    size(trueState)
    err = trueState-state;
    explicit_err = err'*err
    
    % semiimplicit
    maxt = 50;
    dt = 0.03;
    T = 0:dt:maxt+dt;
        trueState = [istate(1) * cos(sqrt(g/L)*T);istate(1) * -sqrt(g/L) * sin(sqrt(g/L)*T)]';

    state = istate;
    for t = 0:dt:maxt
       [theta,omega] = pendulumEuler_semiimplicit(state(end,:),dt);
       state = [state;theta,omega];
    end
    plot(T,state(:,1),'r')

%     plot(T,trueState(:,1),'g');
    
    err = trueState-state;
    semiimplicit_err = err'*err
    
    axis fill;
    %}
    %% Problem 4
    %
    % Double Step Method 
    % ARE EQUATIONS CORRECT?! 
    %{

    function xd = f(x)
        xd = exp(-20*(x-5));
    end
    X = 1;
    dt = 0.01;
    tmax = 0.1;
    tmin = 0;
    for t = tmin:dt:tmax
        % half step
        x1 = X(end) + dt/2 * f(X(end))
        % full step
        x2 = x1 + dt/2 * f(x1)
        % Euler Step
        x  = X(end) + dt*f(X(end))
        e = x2 - x;
        xbar = x+2*e;
        X = [X;xbar];
    end
    T = tmin:dt:tmax+dt;
    plot(T,X,'r');
    plot(T,5 + exp(-20*T),'b')
    axis fill;
    %}
    %% Problem 5%{
    function xdd = f(x,xd)
        xdd = -10000*x - xd;
    end
    
    tmin = 0;
    tmax = 10;
    %{
    %% Explicit Euler 
    X = 1;
    Xd = 0;
    dt = 0.0001;
    T = tmin:dt:tmax;
    for i = 1:numel(T)
        xdd = f(X(end),Xd(end));
        x = X(end) + dt*Xd(end);
        xd = Xd(end) + dt*xdd;
        
        X = [X;x];
        Xd = [Xd;xd];
    end
    plot(T,X(1:end-1),'r');
    hold on;
    %% semi-implicit Euler 
    X = 1;
    Xd = 0;
    dt = 0.01;
    T = tmin:dt:tmax;
    
    for i = 1:numel(T)
        xdd = f(X(end),Xd(end));
        xd = Xd(end) + dt*xdd;
        x = X(end) + dt*xd;
        
        X = [X;x];
        Xd = [Xd;xd];
    end
    plot(T,X(1:end-1),'g');
    axis([0 10 -1 1]);
    %}
    %% RK4
    function qd = qd(state)
        qd = [       state(2)       ;
              -10000*state(1) - state(2)];
    end

    Q = [1;
         0];
    dt = 0.005;
    T = tmin:dt:0.1;
    
    for i = 1:numel(T)
        q = Q(:,end);
        k1 = qd(q);
        k2 = qd(q+dt/2*k1);
        k3 = qd(q+dt/2*k2);
        k4 = qd(q+dt*k3);
        q = q + dt/6*(k1 + 2*k2 + 2*k3 + k4);

        Q = [Q,q];
    end
    [T',Q(:,1:end-1)']
%     plot(T,Q(1,1:end-1),'b');
    axis([0 10 -1 1]);
    
end

