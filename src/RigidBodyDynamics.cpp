#include <Sim/RigidBodyDynamics.h>

using namespace Sim;

Ravelin::Matrix3d& skew(const Vec& v,Ravelin::Matrix3d& M){
  M(0,0) = 0;
  M(0,1) = -v[3];
  M(0,2) =  v[2];
  M(1,0) =  v[3];
  M(1,1) =  0;
  M(1,2) =  -v[1];
  M(2,0) =  -v[2];
  M(2,1) =  v[1];
  M(2,2) =  0;
  return M;
}

void RigidBody::reset(){
  q.set_zero(7);
  v.set_zero(6);
  vd.set_zero(6);

  if(static_){
      // create a GROUND
      vertices.push_back(Ravelin::Vector3d(-2, 0.05, 2));
      vertices.push_back(Ravelin::Vector3d( 2,-0.05, 2));
      vertices.push_back(Ravelin::Vector3d( 2, 0.05, 2));
      vertices.push_back(Ravelin::Vector3d(-2,-0.05, 2));
      vertices.push_back(Ravelin::Vector3d(-2, 0.05,-2));
      vertices.push_back(Ravelin::Vector3d( 2,-0.05,-2));
      vertices.push_back(Ravelin::Vector3d( 2, 0.05,-2));
      vertices.push_back(Ravelin::Vector3d(-2,-0.05,-2));
      q[1] = -0.05; q[6] = 1;
      f.set_zero(6);
      Tws.set_identity(4);
      m = INFINITY;
      Icm.set_identity();
      Icm *= INFINITY;
  } else {
      // create a BOX
      vertices.push_back(Ravelin::Vector3d(-0.5,0.5,0.5));
      vertices.push_back(Ravelin::Vector3d(0.5,-0.5,0.5));
      vertices.push_back(Ravelin::Vector3d(0.5,0.5,0.5));
      vertices.push_back(Ravelin::Vector3d(-0.5,-0.5,0.5));

      vertices.push_back(Ravelin::Vector3d(-0.5,0.5,-0.5));
      vertices.push_back(Ravelin::Vector3d(0.5,-0.5,-0.5));
      vertices.push_back(Ravelin::Vector3d(0.5,0.5,-0.5));
      vertices.push_back(Ravelin::Vector3d(-0.5,-0.5,-0.5));
      q[1] = 1; q[6] = 1;
      v[0] = 1;
      v[4] = 1;
      f.set_zero(6);
      Tws.set_identity(4);
      m = 1;
      Icm.set_identity();
      Icm *= m/6;
  }
  // init Geom Vertices in global
  global_vertices.reserve(vertices.size());
  update();
}

void RigidBody::reference_faces(){
  faces.clear();
  std::vector<Ravelin::Vector3d*> face(4);
  // right
  face[0] = (&global_vertices[2]);
  face[1] = (&global_vertices[1]);
  face[2] = (&global_vertices[5]);
  face[3] = (&global_vertices[6]);
  faces.push_back(face);

  // left
  face[0] = (&global_vertices[3]);
  face[1] = (&global_vertices[0]);
  face[2] = (&global_vertices[4]);
  face[3] = (&global_vertices[7]);
  faces.push_back(face);

  // up
  face[0] = (&global_vertices[0]);
  face[1] = (&global_vertices[4]);
  face[2] = (&global_vertices[6]);
  face[3] = (&global_vertices[2]);
  faces.push_back(face);

  // down
  face[0] = (&global_vertices[1]);
  face[1] = (&global_vertices[5]);
  face[2] = (&global_vertices[7]);
  face[3] = (&global_vertices[3]);
  faces.push_back(face);

  // out
  face[0] = (&global_vertices[1]);
  face[1] = (&global_vertices[2]);
  face[2] = (&global_vertices[0]);
  face[3] = (&global_vertices[3]);
  faces.push_back(face);

  // in
  face[0] = (&global_vertices[5]);
  face[1] = (&global_vertices[6]);
  face[2] = (&global_vertices[4]);
  face[3] = (&global_vertices[7]);
  faces.push_back(face);
}


void RigidBody::apply_force(const Ravelin::Vector3d& fp,
                            const Ravelin::Vector3d& p){
  FILE_LOG(logDEBUG)<<"entered RigidBody::apply_force(.)"<<std::endl;

  FILE_LOG(logDEBUG) << "linear force = " << fp << std::endl;
  FILE_LOG(logDEBUG) << "at point = " << p << std::endl;

  Vec gf(6);
  gf.set_sub_vec(0,fp);
  workv3_ = p;
  workv3_ -= com;
  gf.set_sub_vec(3,Ravelin::Vector3d::cross(workv3_,fp));
  f+=gf;

  FILE_LOG(logDEBUG) << "generalized force = " << gf << std::endl;

  FILE_LOG(logDEBUG)<<"exited RigidBody::apply_force(.)"<<std::endl;
}

Vec& RigidBody::calc_jacobian(const Ravelin::Vector3d& fp,
                              const Ravelin::Vector3d& p,Vec& jrow){
  FILE_LOG(logDEBUG)<<"entered RigidBody::calc_jacobian(.)"<<std::endl;
  jrow.set_sub_vec(0,fp);
  workv3_ = p;
  workv3_ -= com;
  jrow.set_sub_vec(3,Ravelin::Vector3d::cross(workv3_,fp));

//  Ravelin::Matrix3d D,E,F;

//  // Force is at COM and so is the acceleration we want to retrieve c = 0
//  Ravelin::Vector3d c = Ravelin::Vector3d::zero();

//  // D
//  D.set_identity();
//  D*=m;

//  // E
//  E = (skew(c,workM3_) *= m);

//  // F
//  skew(c,workM3_);
//  workM3_.mult(workM3_,F);
//  F *= m;
//  F= -F;
//  F += Icm;

//  Mat A(6,6);
//  A.set_sub_mat(0,0,D);
//  A.set_sub_mat(3,0,E);
//  E = -E;
//  A.set_sub_mat(0,3,E);
//  A.set_sub_mat(3,3,F);

//  Ravelin::Vector3d d, e;

//  // d
//  Ravelin::Vector3d w;
//  v.get_sub_vec(3,6,w);
//  d = Ravelin::Vector3d::cross(w,Ravelin::Vector3d::cross(w,c));
//  d *= m;
//  // e
//  e = Ravelin::Vector3d::cross(w,F.mult(w,workv3_));
//  Vec b(6);
//  b.set_sub_vec(0,d);
//  b.set_sub_vec(3,e);
//  b.negate();
//  b += gf;

//  // solve for acceleration
//  LA_.factor_chol(A);
//  Ravelin::LinAlgd::solve_chol_fast(A,b);

  FILE_LOG(logDEBUG) << "Jacobian Row = " << jrow << std::endl;
  FILE_LOG(logDEBUG)<<"exited RigidBody::calc_jacobian(.)"<<std::endl;

}

Vec& RigidBody::calc_inv_dynamics(const Vec& a,Vec& f,Ravelin::Vector3d c){
    FILE_LOG(logDEBUG)<<"entered RigidBody::calc_inv_dynamics(.)"<<std::endl;

    FILE_LOG(logDEBUG) << "vd (desired) = " << a << std::endl;

//  % F     = total force acting on the center of mass
//  % m     = mass of the body
//  % acm   = acceleration of the center of mass
//  % vcm   = velocity of the center of mass
//  % t     = total torque acting about the center of mass
//  % Icm   = moment of inertia about the center of mass
//  % w     = angular velocity of the body
//  % aa    = angular acceleration of the body

//  |F| = | m*eye(3)         -m*skew(c)       | | acm |
//  |t|   | m*skew(c)  Icm-m*skew(c)*skew(c)] | | aa  |
//        + |       m*cross(w,cross(w,c))          |
//          | cross(w,(Icm - m*skew(c)*skew(c))*w) |

    Ravelin::Matrix3d D,E,F;

    // D
    D.set_identity();
    D*=m;

    // E
    E = (skew(com,workM3_) *= m);

    // F
    skew(com,workM3_);
    workM3_.mult(workM3_,F);
    F *= m;
    F = -F;
    F += Icm;

    Mat A(6,6);
    A.set_sub_mat(0,0,D);
    A.set_sub_mat(3,0,E);
    E = -E;
    A.set_sub_mat(0,3,E);
    A.set_sub_mat(3,3,F);

    Ravelin::Vector3d d,e;
    // d
    Ravelin::Vector3d w;
    v.get_sub_vec(3,6,w);
    d = Ravelin::Vector3d::cross(w,Ravelin::Vector3d::cross(w,com));
    d *= m;
    // e
    e = Ravelin::Vector3d::cross(w,F.mult(w,workv3_));
    Vec b(6);
    b.set_sub_vec(0,d);
    b.set_sub_vec(3,e);

    // calc newton-euler soln
    f = b;

    A.mult(a,f,1,1);
    FILE_LOG(logDEBUG)<<"inv dynamics force = " << f <<std::endl;
    FILE_LOG(logDEBUG)<<"exited RigidBody::calc_inv_dynamics(.)"<<std::endl;
}

void RigidBody::calc_fwd_dynamics(){
FILE_LOG(logDEBUG)<<"entered RigidBody::calc_fwd_dynamics(.)"<<std::endl;
FILE_LOG(logDEBUG)<<"fwd dynamics forces = " << f << std::endl;

//  % F     = total force acting on the center of mass
//  % m     = mass of the body
//  % acm   = acceleration of the center of mass
//  % vcm   = velocity of the center of mass
//  % t     = total torque acting about the center of mass
//  % Icm   = moment of inertia about the center of mass
//  % w     = angular velocity of the body
//  % aa    = angular acceleration of the body

// ( |F| - |       m*cross(w,cross(w,c))          | ) | m*eye(3)         -m*skew(c)       |^-1 =  | acm |
//   |t|   | cross(w,(Icm - m*skew(c)*skew(c))*w) |   | m*skew(c)  Icm-m*skew(c)*skew(c)] |       | aa  |


  Ravelin::Matrix3d D,E,F;

  // Force is at COM and so is the acceleration we want to retrieve c = 0
  Ravelin::Vector3d c = Ravelin::Vector3d::zero();

  // D
  D.set_identity();
  D*=m;

  // E
  E = (skew(c,workM3_) *= m);

  // F
  skew(c,workM3_);
  workM3_.mult(workM3_,F);
  F *= m;
  F= -F;
  F += Icm;

  Mat A(6,6);
  A.set_sub_mat(0,0,D);
  A.set_sub_mat(3,0,E);
  E = -E;
  A.set_sub_mat(0,3,E);
  A.set_sub_mat(3,3,F);

  Ravelin::Vector3d d, e;

  // d
  Ravelin::Vector3d w;
  v.get_sub_vec(3,6,w);
  d = Ravelin::Vector3d::cross(w,Ravelin::Vector3d::cross(w,c));
  d *= m;
  // e
  e = Ravelin::Vector3d::cross(w,F.mult(w,workv3_));
  Vec b(6);
  b.set_sub_vec(0,d);
  b.set_sub_vec(3,e);
  b.negate();
  b += f;

  // solve for acceleration
  LA_.factor_chol(A);
  Ravelin::LinAlgd::solve_chol_fast(A,b);

  FILE_LOG(logDEBUG)<<"fwd dynamics accel = " << b << std::endl;
  add_acceleration(b);
  FILE_LOG(logDEBUG)<<"exited RigidBody::calc_fwd_dynamics(.)"<<std::endl;
}

Mat& calc_q_GL(const Vec& e,Mat& L){
  L.set_zero(3,4);
  // NOTE: [w x y z]

  // G
  L(0,0) = -e[1]; L(0,1) = e[0]; L(0,2) =-e[3]; L(0,3) = e[2];
  L(1,0) = -e[2]; L(1,1) = e[3]; L(1,2) = e[0]; L(1,3) =-e[1];
  L(2,0) = -e[3]; L(2,1) =-e[2]; L(2,2) = e[1]; L(2,3) = e[0];
  // L
//  L(0,0) = -e[1]; L(0,1) = e[0]; L(0,2) = e[3]; L(0,3) =-e[2];
//  L(1,0) = -e[2]; L(1,1) =-e[3]; L(1,2) = e[0]; L(1,3) = e[1];
//  L(2,0) = -e[3]; L(2,1) = e[2]; L(2,2) =-e[1]; L(2,3) = e[0];
}

Vec *vd_;

/// Integratable ODE of the form: void (*f)(double,const Vec&,Vec&)
void fn(double t,const Vec& state,Vec& dstate){
    Vec workv1_,workv2_;
    dstate.set_zero(13);
   // in:  [q,v]'
   // out: [qd,vd]'

    Mat L;
    calc_q_GL(state.get_sub_vec(3,7,workv1_),L);

    // v[xd] -> qd[xd]
    dstate.set_sub_vec(0,state.get_sub_vec(7,10,workv1_));
    // v[w] -> qd[quat]

    dstate.set_sub_vec(3,L.transpose_mult(state.get_sub_vec(10,13,workv1_),workv2_));
    // vd[quat] -> vd[xdd,wd]
    dstate.set_sub_vec(7,*vd_);
}


void RigidBody::step(double dt){
  FILE_LOG(logDEBUG)<<"entered RigidBody::step(double dt)"<<std::endl;

  // set com
  // NOTE: com is transform r in THIS case
  for(unsigned i=0;i<3;i++)
      com[i] = Tws(i,3);

  // NOTE: set accel for integration function
  vd_ = &vd;

  FILE_LOG(logDEBUG) << "q (before) = " << q << std::endl;
  FILE_LOG(logDEBUG) << "v (before) = " << v << std::endl;
  FILE_LOG(logDEBUG) << "vd (before) = " << vd << std::endl;

  // Set forces to apply
  // apply drag force
  {
      double k = 0.1;
      Ravelin::Vector3d drag;
      v.get_sub_vec(0,3,drag);

      drag = -drag *= (drag.dot(drag) * k);
      apply_force(drag,com);
  }

  // apply forces
  calc_fwd_dynamics();

  // Set up state
    Vec state;
    state.set_zero(q.rows() + v.rows());
    state.set_sub_vec(0,q);
    state.set_sub_vec(q.rows(),v);

  // Integrate state forward 1 step
    sim_->integrate(sim_->time,dt,&fn,state);

  // re-normalize quaternion state
    state.get_sub_vec(0,q.rows(),q);
    state.get_sub_vec(q.rows(),state.rows(),v);
    Vec e;
    q.get_sub_vec(3,7,e);
    e.normalize();
    q.set_sub_vec(3,e);

  FILE_LOG(logDEBUG) << "q (after) = " << q << std::endl;
  FILE_LOG(logDEBUG) << "v (after) = " << v << std::endl;
  FILE_LOG(logDEBUG) << "vd (after) = " << vd << std::endl;

  // Update graphics transform
  update();

  FILE_LOG(logDEBUG)<<"exited RigidBody::step(.)"<<std::endl;
  reset_accumulators();
}

Mat& e2R(const Vec& e,Ravelin::Matrix3d& R){
//  Vec e(4);
  R(0,0) = 2*(e[1]*e[1] + e[0]*e[0])-1;
  R(0,1) = 2*(e[1]*e[2] - e[3]*e[0]);
  R(0,2) = 2*(e[1]*e[3] + e[2]*e[0]);

  R(1,0) = 2*(e[1]*e[2] + e[3]*e[0]);
  R(1,1) = 2*(e[2]*e[2] + e[0]*e[0])-1;
  R(1,2) = 2*(e[2]*e[3] - e[1]*e[0]);

  R(2,0) = 2*(e[1]*e[3] - e[2]*e[0]);
  R(2,1) = 2*(e[2]*e[3] + e[1]*e[0]);
  R(2,2) = 2*(e[3]*e[3] + e[0]*e[0])-1;
}

void transform(const Mat& T, Ravelin::Vector3d& v){
  static Vec v41(4), v42(4);
  for(int i=0;i<4;i++)
    v41[i] = (i==3)? 1 : v[i];
  T.mult(v41,v42);
  for(int i=0;i<3;i++)
    v[i] = v42[i];
}

extern std::mutex shapes_mutex;

void RigidBody::update(){
  FILE_LOG(logDEBUG)<<"entered RigidBody::update()"<<std::endl;

  // Set 4x4 T matrix
  Tws.set_identity(4);
  for(unsigned i=0;i<3;i++)
    Tws(i,3) = q[i];
  e2R(q.get_sub_vec(3,7,workv_),workM3_);
//  FILE_LOG(logDEBUG) << "Rws = [\n" << workM3_ << "];" <<std::endl;
  Tws.set_sub_mat(0,0,workM3_);
  FILE_LOG(logDEBUG) << "Tws = [\n" << Tws << "];" <<std::endl;

  // Transform Geom Vertices to global
  shapes_mutex.lock();
  for(unsigned i=0;i<vertices.size();i++){
    global_vertices[i] = vertices[i];
    transform(Tws,global_vertices[i]);
    FILE_LOG(logDEBUG) << "vert[" << i << "] = " << global_vertices[i] <<std::endl;
  }
  reference_faces();
  shapes_mutex.unlock();
  FILE_LOG(logDEBUG)<<"exited RigidBody::update()"<<std::endl;
}


