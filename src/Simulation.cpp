
#include <Sim/Simulation.h>
#include <GL/freeglut.h>
double STEP_SIZE = 0.001;
bool RENDER_SIMULATION = false;
unsigned MAX_ITER = INFINITY,MAX_TIME = INFINITY;

using namespace Sim;

  void Simulation::step(double step_size){
    FILE_LOG(logDEBUG)<<"entered Simulation::step()"<<std::endl;

    unsigned n = rbs_.size();

    Vec grav;
    grav.set_zero(6);
    grav[1] = -9.8;

//    bool UPDATE_SYSTEM = true;
//    while (UPDATE_SYSTEM){
      // apply gravity
      for(int i=0;i<n;i++)
        if(!rbs_[i]->static_)
          rbs_[i]->set_acceleration(grav);

      // Step Boxes
      for(int i=0;i<n;i++)
        if(!rbs_[i]->static_)
          rbs_[i]->step(step_size);

      // check for impacts
      collisions.clear();
      if(collision_check(collisions))
        // Apply contact forces
        penalty_contact(collisions);

//      UPDATE_SYSTEM = false;//
      // resolve impacts
//    }

//    visualize();

    FILE_LOG(logDEBUG)<<"exited Simulation::step()"<<std::endl;
  }

  void Simulation::add_rigid_body(RigidBody* rb){
      rbs_.push_back(rb);
  }

  bool Simulation::collision_check(std::vector<Contact>& collisions){
    FILE_LOG(logDEBUG)<<"entered Simulation::collision_check(.)"<<std::endl;

    // NOTE: Just check for interpenetration between:
    // BOX rbs_[0] and GROUND rbs_[1]
    // just check TOP face of ground
//    std::vector<RigidBody*>& ground_top = rbs_[1].faces[RigidBody::TOP];

    // check which BOX vertex most interpenetrates this surface
    // (pretty much check which vertex has the most negative y)
//    for(unsigned rb = 0;rb<rbs_.size();rb++){
//    FILE_LOG(logDEBUG)<<" Checking " <<  rbs_[0]->global_vertices.size() << " vertices " << std::endl;
//    FILE_LOG(logDEBUG)<<" Checking Vertex 1: " << rbs_[0]->global_vertices[0] <<std::endl;

    for(unsigned v = 0;v<8;v++){
      Ravelin::Vector3d& vert = rbs_[0]->global_vertices[v];
      FILE_LOG(logDEBUG)<<" Checking Vertex: " << vert <<std::endl;

      double depth = -vert[1], last_depth = 0;
      Contact c;
      if(depth>0 && depth > last_depth){
        c.rb1 = rbs_[1];
        c.rb2 = rbs_[0];
        c.normal = Ravelin::Vector3d(0,1,0);
        c.point = vert;
        c.depth = depth;
        collisions.push_back(c);
      }
      last_depth = depth;
    }

    FILE_LOG(logDEBUG)<<"exited Simulation::collision_check(.)"<<std::endl;

    return !collisions.empty();
  }

  void Simulation::penalty_contact(std::vector<Contact>& collisions){
//    for(unsigned i=0;i<collisions.size();i++){
      Contact& c = collisions[0];
      // NOTE: Tunable parameters
      double kp= 100000,kv = -10000,ki=0;
      // calculate magnitude of penalty force
      Vec rvel(6);
      Ravelin::Vector3d cn = c.normal,cvel,
                       tan1(1,0,0), tan2(0,0,1);
      // Create Jacobian
      Mat J(3,6);
      {
        c.rb2->calc_jacobian(c.normal,c.point,workv_);
        J.set_row(0,workv_);
        c.rb2->calc_jacobian(tan1,c.point,workv_);
        J.set_row(1,workv_);
        c.rb2->calc_jacobian(tan2,c.point,workv_);
        J.set_row(2,workv_);

        // re-assign tan1 to direction of contact velocity
        c.rb1->get_velocity(rvel) += c.rb1->get_velocity(workv_);
        J.mult(rvel,cvel);
        tan1[0] = cvel[0];
        tan1[2] = cvel[2];
        tan2 = Ravelin::Vector3d::cross(c.normal,tan1);

        // re-calculate tan jacobians
        c.rb2->calc_jacobian(c.point,tan1,workv_);
        J.set_row(1,workv_);
        c.rb2->calc_jacobian(c.point,tan2,workv_);
        J.set_row(2,workv_);
      }

      J.mult(rvel,cvel);
      cn *= (kp*(c.depth) + kv*(cvel[0]));

      if(!c.rb1->static_)
        c.rb1->apply_force(-cn,c.point);
      if(!c.rb2->static_)
        c.rb2->apply_force(cn,c.point);
//    }
  }

////////////////////////////////////////////////////////////////////////////
// GL functions
#include <GL/glut.h>    // Header File For The GLUT Library
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <unistd.h>     // needed to sleep
#include <Sim/Simulation.h>
#include <Sim/RigidBodyDynamics.h>

static GLfloat VIEWING_DISTANCE_MIN = 3.0f,
                g_fViewDistance = 3 * VIEWING_DISTANCE_MIN,
                g_fViewAngle = 0.0f,
                g_fViewHeight = 1.0f;

static int g_yClick = 0, g_xClick = 0;
static bool g_bButton1Down = false;
bool DISP_FULLSCREEN = false;
/* ASCII code for the escape key. */
#define ESCAPE 27

/* The number of our GLUT window */
int window;

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(int Width, int Height)	        // We call this right after our OpenGL window is created.
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
  glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
  glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				// Reset The Projection Matrix

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

  glMatrixMode(GL_MODELVIEW);
}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
    Height=1;

  glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
  glMatrixMode(GL_MODELVIEW);
}

std::vector< std::vector<std::vector<Ravelin::Vector3d*> > > shapes_;
std::mutex shapes_mutex;
Sim::Simulation* sim;

/* The main drawing function. */
void DrawGLScene()
{
  glLoadIdentity();				// make sure we're no longer rotated.
  glTranslatef(0.0,-g_fViewHeight/50.0,-g_fViewDistance);
  glRotatef(g_fViewAngle,0.0f,1.0f,0.0f);
//  glRotatef(g_fViewHeight,1.0f,0.0f,0.0f);

  shapes_mutex.lock();
  shapes_.clear();
  std::vector<RigidBody*>& rbs = sim->get_rigid_bodies();
  for(int i=0;i<rbs.size();i++)
    shapes_.push_back(rbs[i]->faces);

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	// Clear The Screen And The Depth Buffer
  for(int s=0;s<shapes_.size();s++){
    std::vector<std::vector<Ravelin::Vector3d*> >& shape = shapes_[s];

    // draw a cube (6 quadrilaterals)
    glBegin(GL_QUADS);				// start drawing the cube.

    // top of cube
    for(int f=0;f<6;f++){
      std::vector<Ravelin::Vector3d*>& face = shape[f];
      switch(f){
        case 0:
          glColor3f(0.0f,0.0f,1.0f);			// Set The Color To Blue
          break;
        case 1:
          glColor3f(1.0f,0.5f,0.0f);			// Set The Color To Orange
          break;
        case 2:
          glColor3f(1.0f,0.0f,0.0f);			// Set The Color To Red
          break;
        case 3:
          glColor3f(1.0f,1.0f,0.0f);			// Set The Color To Yellow
          break;
        case 4:
          glColor3f(0.0f,1.0f,0.0f);			// Blue
          break;
        case 5:
          glColor3f(1.0f,0.0f,1.0f);			// Set The Color To Violet
          break;
        default:
          break;
      }
      for(int c=0;c<4;c++){
        Ravelin::Vector3d& corner = *(face[c]);
        glVertex3f(corner[0],corner[1],corner[2]);
      }
    }
    glEnd();					// Done Drawing The Cube
  }
  shapes_mutex.unlock();
  // swap the buffers to display, since double buffering is used.
  glutSwapBuffers();
}

/* The function called whenever a key is pressed. */
void keyPressed(unsigned char key, int x, int y)
{
    /* avoid thrashing this call */
  usleep(100);

    /* If escape is pressed, kill everything. */
    if (key == ESCAPE)
    {
      /* shut down our window */
      glutDestroyWindow(window);

      /* exit the program...normal termination. */
      exit(0);
    } else if (key == 'w') {
      g_fViewDistance -= 1.0f;
    }else if (key == 's') {
      g_fViewDistance += 1.0f;
    }else if (key == 'a') {
      g_fViewAngle += 1.0f;
    }else if (key == 'd') {
      g_fViewAngle -= 1.0f;
    }
}

void MouseButton(int button, int state, int x, int y)
{
  // Respond to mouse button presses.
  // If button1 pressed, mark this state so we know in motion function.

  if (button == GLUT_LEFT_BUTTON)
    {
      g_bButton1Down = (state == GLUT_DOWN) ? true : false;
//      g_yClick = y - 3 * g_fViewDistance;
      g_yClick = y - 3 * g_fViewHeight;
      g_xClick = x - 3 * g_fViewAngle;
    }
}

void MouseMotion(int x, int y)
{
  // If button1 pressed, zoom in/out if mouse is moved up/down.

  if (g_bButton1Down)
    {
//      g_fViewDistance = (y - g_yClick) / 3.0;
      g_fViewHeight = (y - g_yClick) / 3.0;


      g_fViewAngle = (x - g_xClick) / 3.0;
      if (g_fViewDistance < VIEWING_DISTANCE_MIN)
         g_fViewDistance = VIEWING_DISTANCE_MIN;
      glutPostRedisplay();
    }
}


////////////////////////////////////////////////////////////////////////////
//  MAIN FUNCTION

void start_simulation(){
  sim->run(STEP_SIZE,MAX_ITER,MAX_TIME);
}

  int main(int argc,char* argv[]){
      const unsigned ONECHAR_ARG = 3, TWOCHAR_ARG = 4;
      std::string LOG_TYPE;
    // get all options
    for (int i=1; i< argc; i++)
    {
      // get the option
      std::string option(argv[i]);

      // process options
      if (option == "-r"){
        RENDER_SIMULATION = true;
      }
      else if (option == "-s="){
        STEP_SIZE = std::atof(&argv[i][ONECHAR_ARG]);
        assert(STEP_SIZE >= 0.0 && STEP_SIZE < 1);
      }
      else if (option.find("-mi=") != std::string::npos)
      {
        MAX_ITER = std::atoi(&argv[i][TWOCHAR_ARG]);
        assert(MAX_ITER > 0);
      }
      else if (option.find("-mt=") != std::string::npos)
      {
        MAX_TIME = std::atof(&argv[i][TWOCHAR_ARG]);
        assert(MAX_TIME > 0);
      }
      else if (option.find("-l=") != std::string::npos){
          LOG_TYPE = std::atof(&argv[i][ONECHAR_ARG]);

      }
    }
    std::cout << LOG_TYPE << std::endl;
    FILELog::ReportingLevel() = FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");

    sim = new Simulation();
    sim->integrate = RK4::integrate;
    //box
    sim->add_rigid_body(new RigidBody(false,sim));
    //ground
    sim->add_rigid_body(new RigidBody(true,sim));    

    if(RENDER_SIMULATION){
        // set up GLUT
        glutInit(&argc, argv);

        /* Select type of Display mode:
           Double buffer
           RGBA color
           Alpha components supported
           Depth buffered for automatic clipping */
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);

        /* get a 640 x 480 window */
        glutInitWindowSize(640, 480);

        /* the window starts at the upper left corner of the screen */
        glutInitWindowPosition(0, 0);

        /* Open a window */
        window = glutCreateWindow("Jeff Molofee's GL Code Tutorial ... NeHe '99");

        /* Register the function to do all our OpenGL drawing. */
        glutDisplayFunc(&DrawGLScene);

        /* Go fullscreen.  This is as soon as possible. */
        if(DISP_FULLSCREEN)
          glutFullScreen();

        /* Even if there are no events, redraw our gl scene. */
        glutIdleFunc(&DrawGLScene);

        /* Register the function called when our window is resized. */
        glutReshapeFunc(&ReSizeGLScene);

        /* Register the function called when the keyboard is pressed. */
        glutKeyboardFunc(&keyPressed);

        /* Register the function called when the keyboard is pressed. */
        glutMouseFunc (MouseButton);
        glutMotionFunc (MouseMotion);
        /* Initialize our window. */
        InitGL(640, 480);

        std::thread t(start_simulation);

        /* Start Event Processing Engine */
        glutMainLoop();
        t.join();
    } else {
        start_simulation();
    }
    return 0;

  }
