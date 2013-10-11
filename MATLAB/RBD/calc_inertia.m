function [J] = calc_inertia(m,e,x)
    n = numel(m);
    J = zeros(3);
    xbar = mean(x')';
    E = kron(e(:,1),e(:,1)) + kron(e(:,2),e(:,2)) + kron(e(:,3),e(:,3));
    for i = 1:n
        ex = x(:,i) - xbar;
        J = J + reshape(m(i)*((ex'*ex)*E - kron(ex,ex)),3,3);
    end
end

