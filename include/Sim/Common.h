#ifndef _SIM_COMMON_H_
#define _SIM_COMMON_H_
#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Matrix3d.h>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

#include <algorithm>
#include <iostream>
#include <cassert>
#include <vector>
#include <thread>
#include <mutex>

//using std::assert;
using std::vector;
//using std::out;
using std::cerr;
using std::endl;

typedef Ravelin::VectorNd Vec;
typedef Ravelin::MatrixNd Mat;

namespace Sim {
  class RigidBody;
  class Simulation;
  class Integration;
  class Collision;
}

static Ravelin::LinAlgd LA_;

#include <Sim/Log.h>

#endif //_SIM_COMMON_H_
