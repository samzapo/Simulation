#ifndef _SIM_INTEGRATION_H_
#define _SIM_INTEGRATION_H_
#include <Sim/Common.h>
#include <Sim/Simulation.h>
#include <Sim/Collision.h>
#include <Sim/RigidBodyDynamics.h>

namespace Sim{

class Integration{
	public:
    Integration();
    static void integrate(double t,double dt,void (*f)(double,const Vec&,Vec&),Vec& q);
    double dt;
};

class RK4 : public Integration{
  public:
    RK4();
    static void integrate(double t,double dt,void (*f)(double,const Vec&,Vec&),Vec& q);
};

class Euler : public Integration{
  public:
    Euler();
    static void integrate(double t,double dt,void (*f)(double,const Vec&,Vec&),Vec& q);
};

}

#endif //_SIM_INTEGRATION_H_
