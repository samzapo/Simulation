#include <Sim/Integration.h>

using namespace Sim;
//using namespace Opt;

void RK4::integrate(double t,double dt,void (*f)(double,const Vec&,Vec&),Vec& q){
  	// could possibly adaptive time-step
	int n = q.rows();
    Vec k1(2),k2(2),k3(2),k4(2),temp_q;
	f(t,q,k1);
	((temp_q = k1) *= dt/2) += q;
	f(t+dt/2,temp_q,k2);
	((temp_q = k2) *= dt/2) += q;
	f(t+dt/2,temp_q,k3);
	((temp_q = k3) *= dt) += q;
	f(t+dt,temp_q,k4);
    
	for(int i=0;i<n; i++)
		q[i] = q[i] + dt/6*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
}


///// Test Functions
//void f(double t,const Vec& q, Vec& qd){
//	qd[0] = q[1];
//	qd[1] = -10000*q[0] - q[1];
//}

//int main(int argc,char* argv[]){
//    RK4 rk_;
//	Vec q(2);
//	q[0] = 1.0;
//	q[1] = 0.0;
//	double dt = 0.005,tmin=0, tmax=0.1;
//	for(double t=tmin;t<=tmax;t+=dt){
//	   FILE_LOG(logDEBUG) << t << " " << q[0] << " " << q[1] << std::endl;
//	   rk_.integrate(t,dt,&f,q);
//	}
//}
