#include <math.h>
#include <assert.h>

#ifdef WIN32
#include <windows.h>
#endif

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#include <GL/GL.h>
#include <GL/glut.h>
#include <GL/glui.h>

#include <string.h>

#include <HL/hl.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#include <vector>
#include <iostream>
#include <memory>

using namespace std;

static const double kPI = 3.1415926535897932384626433832795;

static hduVector3Dd gCameraPosWC;
static int gWindowWidth, gWindowHeight;
static int gViewportWidth, gViewportHeight;

#define CURSOR_SIZE_PIXELS 20
static double gCursorScale;
static GLuint gCursorDisplayList = 0;

/* Variables used by the trackball emulation. */
static hduMatrix gCameraRotation;
static double gCameraScale = 1.0;
static double gCameraTranslationX = 0;
static double gCameraTranslationY = 0;
static bool gIsRotatingCamera = false;
static bool gIsScalingCamera = false;
static bool gIsTranslatingCamera = false;
static int gLastMouseX, gLastMouseY;

/* Function prototypes. */
void glutDisplay();
void glutReshape(int width, int height);
void glutIdle(); 
void glutMouse(int button, int state, int x, int y);
void glutMotion(int x, int y);
void glutKeyboard(unsigned char key, int x, int y);

void initScene();
void initGL();
void initHL();
void updateCamera();
void updateWorkspace();
void drawCursor();
void drawScene();
void __cdecl exitHandler();


/* Haptic device and rendering context handles. */
static HHD ghHD = HD_INVALID_HANDLE;
static HHLRC ghHLRC = NULL;

/* Live variables passed into GLUI. */
int main_window;

/* Lighting parameters */
GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
GLfloat light0_diffuse[] =  {.6f, 1.0f, 0.1f, 1.0f};
GLfloat light0_position[] = {.5f, .5f, 1.0f, 0.0f};

GLfloat light1_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
GLfloat light1_diffuse[] =  {1.0f, 0.1f, 0.4f, 1.0f};
GLfloat light1_position[] = {-1.0f, 0.9f, 1.0f, 0.0f};

/* Initialize mesh */
HLuint shapeId_cone;
HLuint effect;//custom force

/* moving cube current position*/
Eigen::Vector3f cube_current;

/* particle position&velocity at t*/
Eigen::Matrix3f p_mass;
Eigen::Matrix3f k_stiff;
Eigen::Matrix3f damping;
float particle_mass;
const float Kcube = 10;
const float Khaptic = 10;
Eigen::Vector3f f0;
bool flag;

Eigen::Matrix4f modelwork;//Matrix mapping from model space to haptics workspace
Eigen::Matrix4f workspacemodel;//Matrix mapping from haptics workspace to model

struct PointMass //for custom effect for haptic device
{
    Eigen::Vector3f m_position;//current position p0
    Eigen::Vector3f m_velocity;//v0
    HDdouble m_mass; //particle mass
    
	PointMass(){}
	PointMass(Eigen::Vector3f pos,Eigen::Vector3f vel,HDdouble m){
	m_position=pos;
	m_velocity=vel;
	m=m_mass;
	}
};


PointMass particleMass;//particle mass
shared_ptr<PointMass> global_particle;//write from the haptic thread and read from the graphic thread

hduVector3Dd global_proxyPos;

//custom force effect
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata);
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata);
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata);
void initPointMass(PointMass *pPointMass);


int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(850, 500);

    main_window = glutCreateWindow("MuscleMass Haptics");
  

    // Set glut callback functions.
    glutDisplayFunc(glutDisplay);
    glutMotionFunc(glutMotion);
    glutMouseFunc(glutMouse);
    glutReshapeFunc(glutReshape);
	glutKeyboardFunc(glutKeyboard);
    
    // The GLUT main loop won't return control, so we need to perform cleanup
    // using an exit handler.
    atexit(exitHandler);
    
	//init GL and HL
    initScene(); 
    
    // egister the idle callback with GLUI (not with GLUT).
    glutIdleFunc(glutIdle);

    glutMainLoop();

    
	//CustomForce end
	hlBeginFrame();
    hlStopEffect(effect);
    hlEndFrame();

    return 0;
}


void exitHandler()
{
    // Shutdown the haptic mouse.
    //hmShutdownMouse();
	hlDeleteEffects(effect,1);

    // Free up the haptic rendering context.
    hlMakeCurrent(NULL);
    if (ghHLRC != NULL)
    {
        hlDeleteContext(ghHLRC);
    }

    // Free up the haptic device.
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
    }
}

void initScene(){
	initGL();
	initHL();
}

void initGL()
{
    glClearColor(0.9f, 1.0f, 0.5f, 1.0f);
    // Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    
    // Cull back faces.
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    
	glShadeModel(GL_SMOOTH);

    //// Set lighting parameters.
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

	particle_mass=10; 
	//Mass Matrix
	p_mass<<particle_mass,0,0,
		    0,particle_mass,0,
			0,0,particle_mass;

	//Identity Matrix
	Eigen::Matrix3f Identity_m;
	Identity_m<<1,0,0,
		        0,1,0,
				0,0,1;

	//stiffness
	k_stiff=-1*(Kcube+Khaptic)*Identity_m;

	//damping matrix
	damping<<1,0,0,
		     0,1,0,
			 0,0,1;

	//the cube will be in the center of the screen when running
	flag=false;
	cube_current<<0,0,0; 
}

void initHL()
{
    HDErrorInfo error;
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "Press any key to exit");
        getchar();
        exit(1);
    }   
    
    // Create a haptic context for the device.  The haptic context maintains 
    // the state that persists between frame intervals and is used for
    // haptic rendering.
    ghHLRC = hlCreateContext(ghHD);
    hlMakeCurrent(ghHLRC); 
	
    // Enable optimization of the viewing parameters when rendering
    // geometry for OpenHaptics.
    hlEnable(HL_HAPTIC_CAMERA_VIEW);

	// Generate id for the shape.
	shapeId_cone=hlGenShapes(1);

	//CustomForce Effect
	effect = hlGenEffects(1);        
    // Initialize the point mass.
    initPointMass(&particleMass);
	
	//init global particle
	Eigen::Vector3f init_pos;
	init_pos<<0,0,0;
	Eigen::Vector3f init_velo;
	init_velo<<0,0,0;
	global_particle=make_shared<PointMass>(init_pos,init_velo,0);

	//Start the custom effect callback functions
    hlBeginFrame();

    hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc) computeForceCB, &particleMass);
    hlCallback(HL_EFFECT_START, (HLcallbackProc) startEffectCB, &particleMass);
    hlCallback(HL_EFFECT_STOP, (HLcallbackProc) stopEffectCB, &particleMass);

    hlStartEffect(HL_EFFECT_CALLBACK, effect);

    hlEndFrame();


}

//From example code: ShapeManipulation
//GLUT callback for reshaping the window.This is the main place where the viewing and workspace transforms get initialized.
void glutReshape(int width, int height)
{
    static const double kFovY = 40;
    static const double kCanonicalSphereRadius = 2;

    glViewport(0, 0, width, height);
    gWindowWidth = width;
    gWindowHeight = height;

    // Compute the viewing parameters based on a fixed fov and viewing
    // sphere enclosing a canonical box centered at the origin.

    double nearDist = kCanonicalSphereRadius / tan((kFovY / 2.0) * kPI / 180.0);
    double farDist = nearDist + 2.0 * kCanonicalSphereRadius;
    double aspect = (double) width / height;
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(kFovY, aspect, nearDist, farDist);


    // Place the camera down the Z axis looking at the origin.
    gCameraPosWC[0] = 0;
    gCameraPosWC[1] = 0;
    gCameraPosWC[2] = nearDist + kCanonicalSphereRadius;
 
    updateCamera();
}

//From example code: ShapeManipulation 
//Set the modelview transform from scratch. Apply the current view orientation and scale. 
void updateCamera()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();            
    gluLookAt(gCameraPosWC[0], gCameraPosWC[1], gCameraPosWC[2],
              0, 0, 0,
              0, 1, 0);
    
    glTranslatef(gCameraTranslationX, gCameraTranslationY, 0);
    glMultMatrixd(gCameraRotation);
    glScaled(gCameraScale, gCameraScale, gCameraScale);

    updateWorkspace();

    glutPostRedisplay();
}

//Use the current OpenGL viewing transforms to initialize a transform for the haptic device workspace so that it's properly mapped to world coordinates. 
void updateWorkspace(){
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    hlMatrixMode(HL_TOUCHWORKSPACE);
    hlLoadIdentity();
    
    // Fit haptic workspace to view volume.
    hluFitWorkspace(projection);

    // Compute cursor scale.
    gCursorScale = hluScreenToModelScale(modelview, projection, (HLint*)viewport);
    gCursorScale *= CURSOR_SIZE_PIXELS;


	//Generates matrix that transforms from model to workspace coordinates
	HLdouble viewtouch[16];
	HLdouble touchworkspace[16];
	HLdouble modelworkspace[16];//matrix that transforms from model to workspace coordinates

	hlGetDoublev(HL_VIEWTOUCH_MATRIX,viewtouch);
	hlGetDoublev(HL_TOUCHWORKSPACE_MATRIX,touchworkspace);
	hluModelToWorkspaceTransform(modelview,viewtouch,touchworkspace,modelworkspace);

	
	modelwork<<modelworkspace[0],modelworkspace[4],modelworkspace[8],modelworkspace[12],
		       modelworkspace[1],modelworkspace[5],modelworkspace[9],modelworkspace[13],
			   modelworkspace[2],modelworkspace[6],modelworkspace[10],modelworkspace[14],
			   modelworkspace[3],modelworkspace[7],modelworkspace[11],modelworkspace[15];

	workspacemodel=modelwork.inverse();

}

void glutDisplay(){
	drawScene();
}

void drawScene(){
	//haptics frame
	float time = glutGet(GLUT_ELAPSED_TIME);
	float radian=time/180*kPI/50;
	if(flag)
		cube_current<<2*cos(radian),2*sin(radian),0; //it is circling around the origin with radius 2
	else
		cube_current<<0,0,0;

	hlBeginFrame();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);   

	hlCheckEvents();
    hlTouchModel(HL_CONTACT);
    hlTouchableFace(HL_FRONT);
	
	//draw cube
	glPushMatrix();
      glTranslatef(cube_current(0),cube_current(1),cube_current(2));
	  glutSolidCube(0.5);
      //Make sure proxy resolution is on.  The event handler
      //turns it off but it must be on for shapes to be felt.
      hlEnable(HL_PROXY_RESOLUTION);
	  hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, shapeId_cone);
	  hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.8f);
	  hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.0f);
      hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.1f);
      hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.5f);
	  glutSolidCube(0.5);
	  hlEndShape();
	glPopMatrix();

	//drawing particle (reading from global particle)
	glPushMatrix();
	  glTranslatef(global_particle->m_position(0),global_particle->m_position(1),global_particle->m_position(2));
	  glutSolidSphere(0.1, 32, 32); 
	glPopMatrix();


	hduVector3Dd ProxyPos;
    hduQuaternion ProxyRot;

	hlGetDoublev(HL_PROXY_POSITION, ProxyPos);
    hlGetDoublev(HL_PROXY_ROTATION, ProxyRot);

	//drawing lines between particle cube and cursor
	glBegin(GL_LINES); 
       {
		glColor3f(1,0,0);
        glVertex3f(global_particle->m_position(0),global_particle->m_position(1),global_particle->m_position(2));
		glColor3f(1,0,0);
        glVertex3f(ProxyPos[0],ProxyPos[1],ProxyPos[2]);
		glColor3f(1,0,0);
        glVertex3f(global_particle->m_position(0),global_particle->m_position(1),global_particle->m_position(2));
		glColor3f(1,0,0);
        glVertex3f(cube_current(0),cube_current(1),cube_current(2));

       } 
        glEnd();
        glEndList();

	drawCursor(); //draw cursor on the screen

	glutSwapBuffers();

	hlEndFrame();

}


//From example code: ShapeManipulation.  Displays a cursor using the current haptic device proxy transform and the mapping between the workspace and world coordinates
void drawCursor()
{
    static const double kCursorRadius = 0.5;
    static const int kCursorTess = 15;
    HLdouble proxytransform[16];
    
    GLUquadricObj *qobj = 0;
    
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();
    
    if (!gCursorDisplayList)
    {
        gCursorDisplayList = glGenLists(1);
        glNewList(gCursorDisplayList, GL_COMPILE);
        qobj = gluNewQuadric();
        
        gluSphere(qobj, kCursorRadius, kCursorTess, kCursorTess);
        
        gluDeleteQuadric(qobj);

	   // Draw three lines - one along each coordinate axis from -1 to 1.
        glBegin(GL_LINES); 
       {
		glColor3f(1,0,0);
        glVertex3f(-1.5,0,0);
		glColor3f(1,0,0);
        glVertex3f(1.5,0,0);
		glColor3f(0,1,0);
        glVertex3f(0,-1.5,0);
		glColor3f(0,1,0);
        glVertex3f(0,1.5,0);
		glColor3f(0,0,1);
        glVertex3f(0,0,-1.5);
		glColor3f(0,0,1);
        glVertex3f(0,0,1.5);
       } 
        glEnd();
        glEndList();
    }  
    
    // Apply the local position/rotation transform of the haptic device proxy.
    hlGetDoublev(HL_PROXY_TRANSFORM, proxytransform);
    glMultMatrixd(proxytransform);
    
    // Apply the local cursor scale factor.
    glScaled(gCursorScale, gCursorScale, gCursorScale);
    
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.0, 0.5, 1.0);
    
    glCallList(gCursorDisplayList);
    
    glPopMatrix(); 
    glPopAttrib();
}

void glutIdle()
{
    // According to the GLUT specification, the current window is 
    // undefined during an idle callback.  So we need to explicitly change
    // it if necessary.
    if (glutGetWindow() != main_window) 
    {
        glutSetWindow(main_window);
    }
    
    glutPostRedisplay();
     
}


/*From example code: ShapeManipulation  GLUT callback for responding to mouse button presses.  
Detect whether to initiate a point snapping, view rotation or view scale.*/
void glutMouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            gIsRotatingCamera = true;            
        }
        else if (button == GLUT_RIGHT_BUTTON)
        {
            gIsScalingCamera = true;            
        }
        else if (button == GLUT_MIDDLE_BUTTON)
        {
            gIsTranslatingCamera = true;
        }

        gLastMouseX = x;
        gLastMouseY = y;
    }
    else
    {
        gIsRotatingCamera = false;
        gIsScalingCamera = false;
        gIsTranslatingCamera = false;
    }
}

/*From example code: ShapeManipulation This routine is used by the view rotation code for simulating a virtual
 trackball.  This math computes the z height for a 2D projection onto the
 surface of a 2.5D sphere.  When the input point is near the center of the
 sphere, this routine computes the actual sphere intersection in Z.  When 
 the input point moves towards the outside of the sphere, this routine will 
 solve for a hyperbolic projection, so that it still yields a meaningful answer.*/
double projectToTrackball(double radius, double x, double y)
{
    static const double kUnitSphereRadius2D = sqrt(2.0);
    double z;

    double dist = sqrt(x * x + y * y);
    if (dist < radius * kUnitSphereRadius2D / 2.0)
    {
        // Solve for sphere case.
        z = sqrt(radius * radius - dist * dist);
    }
    else
    {
        // Solve for hyperbolic sheet case.
        double t = radius / kUnitSphereRadius2D;
        z = t * t / dist;
    }

    return z;
}

/*From example code: ShapeManipulation.  GLUT callback for mouse motion, which is used for controlling the view
 rotation and scaling.*/ 
void glutMotion(int x, int y)
{
    if (gIsRotatingCamera)
    {
        static const double kTrackBallRadius = 0.8;   

        hduVector3Dd lastPos;
        lastPos[0] = gLastMouseX * 2.0 / gWindowWidth - 1.0;
        lastPos[1] = (gWindowHeight - gLastMouseY) * 2.0 / gWindowHeight - 1.0;
        lastPos[2] = projectToTrackball(kTrackBallRadius, lastPos[0], lastPos[1]);

        hduVector3Dd currPos;
        currPos[0] = x * 2.0 / gWindowWidth - 1.0;
        currPos[1] = (gWindowHeight - y) * 2.0 / gWindowHeight - 1.0;
        currPos[2] = projectToTrackball(kTrackBallRadius, currPos[0], currPos[1]);

        currPos.normalize();
        lastPos.normalize();

        hduVector3Dd rotateVec = lastPos.crossProduct(currPos);
        
        double rotateAngle = asin(rotateVec.magnitude());
        if (!hduIsEqual(rotateAngle, 0.0, DBL_EPSILON))
        {
            hduMatrix deltaRotation = hduMatrix::createRotation(
                rotateVec, rotateAngle);            
            gCameraRotation.multRight(deltaRotation);
        
            updateCamera();
        }
    }
    if (gIsTranslatingCamera)
    {
        gCameraTranslationX += 10 * double(x - gLastMouseX)/gWindowWidth;
        gCameraTranslationY -= 10 * double(y - gLastMouseY)/gWindowWidth;

        updateCamera();
    }
    else if (gIsScalingCamera)
    {
        float y1 = gWindowHeight - gLastMouseY;
        float y2 = gWindowHeight - y;

        gCameraScale *= 1 + (y1 - y2) / gWindowHeight;  

        updateCamera();
    }

    gLastMouseX = x;
    gLastMouseY = y;
}


//Haptic servo loop which computes force and renders to the haptic device
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata)
{
    PointMass *pPointMass = static_cast<PointMass *>(userdata);

    // Get the time delta since the last update.
	HDdouble instRate;
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
    float h=1e-2;
    // Get the current proxy position from the state cache.
    // Note that the effect state cache is maintained in workspace coordinates,
    // so we don't need to do any transformations in using the proxy
    // position for computing forces.
    hduVector3Dd proxyPos;
	hlCacheGetDoublev(cache,HL_PROXY_POSITION,proxyPos);
	
	//transform the state cache proxy position from workspace to model 
	Eigen::Vector4f cursorP;
	cursorP<<proxyPos[0],proxyPos[1],proxyPos[2],1;
	Eigen::Vector4f result_mw=workspacemodel*cursorP;
	proxyPos[0]=result_mw[0];
	proxyPos[1]=result_mw[1];
	proxyPos[2]=result_mw[2];

	//Implicit Integration Calculations
    Eigen::Vector3f cube_force;
    Eigen::Vector3f haptic_force;
	Eigen::Vector3f net_force;
	cube_force=Kcube*(cube_current-pPointMass->m_position);
	haptic_force<<Khaptic*(proxyPos[0]-pPointMass->m_position(0)),Khaptic*(proxyPos[1]-pPointMass->m_position(1)),Khaptic*(proxyPos[2]-pPointMass->m_position(2));
    net_force=cube_force+haptic_force;
	
	Eigen::Vector3f particle_v1;
    Eigen::Matrix3f middle_value=p_mass-pow(h,2)*k_stiff+h*damping;
	particle_v1=middle_value.inverse()*(p_mass*pPointMass->m_velocity+h*net_force);

	Eigen::Vector3f particle_nextPos = pPointMass->m_position+h*particle_v1;


   //update current position and velocity 
	pPointMass->m_velocity=particle_v1;
	pPointMass->m_position=particle_nextPos;

	//write into the global particle
	global_particle->m_velocity=particle_v1;
	global_particle->m_position=particle_nextPos;

	//Adjust the forces for feeling in the haptic device
    double force_x=haptic_force(0)/4;
	double force_y=haptic_force(1)/4;
	double force_z=haptic_force(2)/4;

	if(force_x>1.0)
		force_x=1.0;
	if(force_y>1.0)
		force_y=1.0;
	if(force_z>1.0)
		force_z=1.0;

    force[0] = -force_x;
    force[1] = -force_y;
    force[2] = -force_z;
	
}

void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
    PointMass *pPointMass = (PointMass *) userdata;
    
    fprintf(stdout, "Custom effect started\n");

	pPointMass->m_position<<0,0,0;
    pPointMass->m_velocity<<0,0,0;
}

void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
    fprintf(stdout, "Custom effect stopped\n");
}

void initPointMass(PointMass *pPointMass)//init the particle 
{
	pPointMass->m_mass = particle_mass;//same as particle_mass
	    
}

//When press "r", the cube will circle around the origin
void glutKeyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case 'r':
        flag=!flag;
        break;
    }
}
