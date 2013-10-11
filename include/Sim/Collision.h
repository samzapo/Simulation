#ifndef _SIM_COLLISION_H_
#define _SIM_COLLISION_H_
#include <Sim/Common.h>
#include <Sim/Simulation.h>
#include <Sim/Integration.h>
#include <Sim/RigidBodyDynamics.h>

class CollisionParams {
    public:
       /// (n x 6) Normal Jacobian
 	   Mat N;

     /// and its n x 6 time derivative
 	   Mat Ndot;
 
 	   /// (n*nk x 6) Tangent Jacobain (polygonal)
 	   Mat D;
 	   /// and its n*nk x 6 time derivative
 	   Mat Ddot;
 	   
 	   /// (n x 6) Sliding Jacobian (Tangent jacobian in direction of sliding)
 	   Mat Q;
 
 	   /// (n)x(3) Points of collision (in global coordinate frame)
 	   std::vector<Ravelin::Vector3d> cp;
 
 	   /// (6 x 1) Pre-collision velocity
 	   Vec Vi;
 
 	   /// (6 x 1) pre-collision acceleration
 	   Vec a_;
 
 	   /// (6 x 6) Inertia Matrix
 	   Vec M;
};

/// Virtual Class for all Collision 
class Collision {
	public:
		int check_for_collisions(std::vector<CollisionParams>&);
	    bool solve_collision(const CollisionParams&,Vec&,Vec&);    
};

/// Class for resolving forces on bodies Undergoing Resting contact
class RestingContact : public Collision {
	public:
	    bool solve_collision(const CollisionParams&,Vec& f,Vec& a);
};

/// Class for resolving impulses of bodies undergoing Impact (or unsolvable resting contact)
class Impact : public Collision {
    public:
        bool solve_collision(const CollisionParams&,Vec& p,Vec& dv);
};

#endif //_SIM_COLLISION_H_
