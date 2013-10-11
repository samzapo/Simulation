#ifndef SIMULATION_H
#define SIMULATION_H
#include <Sim/Common.h>
#include <Sim/Integration.h>
#include <Sim/Collision.h>
#include <Sim/RigidBodyDynamics.h>

namespace Sim{
  class Simulation{
    public:
      Simulation(){
        time = 0;
        iter = 0;
      }

      void add_rigid_body(RigidBody* rb);

      std::vector<RigidBody*>& get_rigid_bodies(){ return rbs_; }

      void run(double step_size,unsigned max_iter,double max_time){
        while(time < max_time && iter < max_iter){
            FILE_LOG(logINFO)<<"Time : " << time << ", Iter : " << iter <<std::endl;

          step(step_size);
          iter ++;
          time += step_size;
          usleep(1000000*step_size);
        }
      }

      /// Simulation time
      double time;


      /// Pointer to user specified Integrator
      void (*integrate)(double,double,void (*f)(double,const Vec&,Vec&),Vec&);

      /// Display the simulator using OSG

      /// Proceed the simulatior
      void step(double step_size);

    private:
      /// Pointer to bodies in the simulation;
      std::vector<RigidBody*> rbs_;

      struct Contact{
        RigidBody *rb1,*rb2;

        /// Normal wrt out of rb2 face
        Ravelin::Vector3d normal,point;
        double depth;
      };

      int iter;

      std::vector<Contact> collisions;
      bool collision_check(std::vector<Contact>& collisions);
      void penalty_contact(std::vector<Contact>& collisions);

//      void visualize();
      void init_visualize();

      /// Work Mats
      Ravelin::Vector3d workv3_;
      Ravelin::Matrix3d workM3_;
      Mat workM_;
      Vec workv_;
  };


}

#endif // SIMULATION_H
