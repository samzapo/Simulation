function CFRestingContact
 nc = 1;
 % 2d circle contacting a plane 
 mu = 0.1;
 % mass
 m = 1;
 % length between end of rod and COM
 r = 1;
 % angle between rod and surface
 theta = 1.1;
 % gravity
 g = 9.8;
 % rotational inertia
 J = pi/4 * r^4;
 % a-
 a_ = [0 -g 0]';
 % intertia
 M =[m 0 0;
     0 m 0; 
     0 0 J];
 
 % jacobians
 N = [0 1 0];
 S = [1 0 r];
 
 
 R = [N; S];
 
 %% LCP formulation
  fprintf('LCP Formulation\n');

 % N*inv(M) * (µ*S' - N')
 A = [N/M * -(mu*S'-N')];
 % N*a- + Ndot*v
 q = [N*a_ + 0]; 
 
 % [N*inv(M) * (µ*S' - N')] [cN] + [N*a- + Ndot*v] = [?]
 [cN,alpha] = lemke(A,q,[0])

 % compute frictional force
 cD = -cN*mu;
 
 a = inv(M)*R'*[cN cD]' + a_
 N*a
 
 %% LP Formulation
 
 G = [N/M*R'];
 A = [ 1 0 ;
       G  ];
 b = [0,0]';
 
 fprintf('LP Formulation\n');
 x = linprog(G,A,b)


 
 a = inv(M)*R'*[cN cD]' + a_
 N*a
  
  %% QP Formulation

 fprintf('QP Formulation\n');
 A = [ 1 0 ;
       G  ];
 b = [0,0]';
 
 function out = f(in)
       out = in(1:nc)*N*(inv(M)*((N'-mu*S')*in(1:nc))+a_);
 end

 x0 = zeros(nc*2,1);
 % cN`*N*(inv(M)*((N`-mu*S`)*cN)+a-)
 x = fmincon(@f,x0,A,b)
 
  a = inv(M)*R'*[cN cD]' + a_
  N*a
  
  fprintf('The objective function [cN`*N*(inv(M)*((N`-mu*S`)*cN)+a-)] leaves cS as a free variable,\n meaning we could potentially have excessive energy gain in the contacting system\n');

end

