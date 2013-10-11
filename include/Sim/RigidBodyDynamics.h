#ifndef RIGIDBODYDYNAMICS_H
#define RIGIDBODYDYNAMICS_H
#include <Sim/Common.h>
#include <Sim/Simulation.h>
#include <Sim/Integration.h>
#include <Sim/Collision.h>
namespace Sim{

  class RigidBody{
    public:
      RigidBody(bool stat = false,Simulation* sim = NULL){
        sim_ = sim;
        static_ = stat;
        reset();
      }
      RigidBody(Simulation* sim, osg::Shape* shape, bool stat = false){
        sim_ = sim;
        shape_ = shape;
        static_ = stat;
        reset();
      }

      void step(double dt);

      /// Return the generalized coordinates of the Rigid Body
      Vec& get_coordinates(Vec& q){ q = this->q;return q; }

      /// Set the generalized coordinates of the Rigid Body
      void set_coordinates(const Vec& q){this->q = q;}

      /// Return the generalized velocity of the Rigid Body
      Vec& get_velocity(Vec& v){v = this->v;return v; }

      /// Set the generalized velocity of the Rigid Body
      void set_velocity(const Vec& v){this->v = v;}

      /// Return the generalized acceleration of the Rigid Body
      Vec& get_acceleration(Vec& vd){ vd = this->vd;return vd; }

      /// Set the generalized velocity of the Rigid Body
      void add_acceleration(const Vec& vd){this->vd += vd;}

      /// Set the generalized velocity of the Rigid Body
      void set_acceleration(const Vec& vd){this->vd = vd;}

      /// apply additional generalized forces to the Rigid Body
      void apply_force(const Vec& f){this->f += f;}
      void apply_force(const Ravelin::Vector3d& f,const Ravelin::Vector3d& p);

      /// apply additional generalized forces to the Rigid Body
      void reset_accumulators(){
          vd.set_zero();
          f.set_zero();
      }

      /// Return all existing forces acting on Rigid Body
      Vec& get_forces(Vec& f){f = this->f;return f;}

      void update();

      Vec& calc_inv_dynamics(const Vec& a, Vec& f, Ravelin::Vector3d c = Ravelin::Vector3d::zero());
      Vec& calc_jacobian(const Ravelin::Vector3d& fp,const Ravelin::Vector3d& p,Vec& jrow);
      std::vector<Ravelin::Vector3d> vertices;
      std::vector<Ravelin::Vector3d> global_vertices;

      enum Faces{
        RIGHT = 0,
        LEFT,
        TOP,
        BOTTOM,
        FRONT,
        BACK
      };

      std::vector<std::vector<Ravelin::Vector3d*> > faces;

      osg::Shape* shape_;

      bool static_;

      ///  Inertias
      Ravelin::Matrix3d Icm;
      Ravelin::Vector3d com;
      double m;

      /// State Vectors [ x y z e1 e2 e3 e4 ]'
      Vec q;

      /// dState Vectors [ x y z ax ay az ]'
      Vec v,vd;

      /// State Vectors [ F , tau ]'
      Vec f;

      /// Graphics Transforms
      Mat Tws;

  private:
     Simulation* sim_;
     void calc_fwd_dynamics();
     void reference_faces();
      /// Work Matrices
      Ravelin::Vector3d workv3_;
      Ravelin::Matrix3d workM3_;
      Mat workM_;
      Vec workv_;
      void reset();

  };
}

#endif // RIGIDBODYDYNAMICS_H
