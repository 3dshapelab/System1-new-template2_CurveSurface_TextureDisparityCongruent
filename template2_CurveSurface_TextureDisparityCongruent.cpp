// Template goal: build curved surface vertices; map 2D image as texture;

// The default is to use vector to store vertices info. 
// If you want to use array intead, uncomment some lines.


// set resetScreen_betweenRuns = false to stop the monitor from moving between runs
// this script assume the stimulus is horizontal, meaning z is a function of y only.



#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
//#include "Optotrak.h"
//#include "Optotrak2.h"
#include "Marker.h"
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
//#include "CoordinatesExtractor.h"
//#include "CylinderPointsStimulus.h"
//#include "EllipsoidPointsStimulus.h"
//#include "StimulusDrawer.h"
#include "GLText.h"
#include "Util.h"
#include "SOIL.h"
#define BROWN 
#ifdef BROWN
#include "BrownMotorFunctions.h"
#else
#include "RoveretoMotorFunctions.h"
#endif
/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;

#include "Calibration_017B.h"
const float DEG2RAD = M_PI / 180;

/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode = true;
static const bool stereo = true;
bool resetScreen_betweenRuns = false;

/********* VARIABLES OBJECTS  **********************/
Screen screen;
VRCamera cam;
//Optotrak2 *optotrak;
//CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60;

/********* VISUAL STIMULI *********/
double display_distance = -400;
double visual_angle = 8; // stim diangonal size
double stimulus_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));
double stimulus_width = 1.2 * stimulus_height;
// stimulus depth
double depth = 10;
double depth_step = 5;

/********* TEXTURE *********/
double normalizer_base = 90; // maximum distance along the side on the sine wave fluctuating in depth
double normalizer = normalizer_base; //will vary based on display distance
double normalizer_u = normalizer_base;
double normalizer_v = normalizer_base;
//TEXTURE LOADING VARIABLE
GLuint loadedTexture[5];
int texnum = 0;


/********* VERTICES *********/
double nr_points_width = 201; // nr of points in x direction
double nr_points_height = 201;
int total_ind = 0;
// Cylinder composed of triangles (Method 1, recommended)

/*
// uncomment the following if prefer using array to store data (vs. vector)
GLfloat vertices[50000000]; // the vertices on the base(e.g. texture) surfacebuildVertices1
GLfloat texcoors[50000000]; // for uv coordinates on the texture mapping
GLfloat colors[50000000];//color arrays, where texture is defined
GLfloat normals[50000000];
GLuint indices_draw_triangle[50000000]; // tracking indices number
*/


struct VerticesData {
	std::vector<GLfloat> vertices_vec;
	std::vector<GLfloat> texcoors_vec;
	std::vector<GLfloat> colors_vec;
	std::vector<GLfloat> normals_vec;
	std::vector<GLuint> indices_draw_triangle_vec;
};
VerticesData my_vertices;

/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void update(int value);
void idle();
void initMotors();
void initGLVariables();
void initVariables();
void initStreams();
void initRendering();
void initStimulus();
void drawStimulus();
double calculateStimulus_ZfromY(double cylDepth, double vertexY);
void buildVertices_triangles(double cylinderDepth, VerticesData& vertices_data);
void drawVertices_triangles(int texNum, const VerticesData& vertices_data);
int LoadGLTextures();
void drawInfo();

/*************************** FUNCTIONS ***********************************/
/***** SOUND THINGS *****/
void beepOk(int tone) {
	switch (tone) {
	case 0:
		// Remember to put double slash \\ to specify directories!!!
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-1.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 1:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-6.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	}
	return;
}

void drawInfo() {
	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_BLEND);
	GLText text;
	if (gameMode)
		text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
	else
		text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

	text.enterTextInputMode();
	text.draw(" IOD: " + stringify<double>(interoculardistance) + "     --- Press M N");
	text.draw(" depth: " + stringify<double>(depth) + "     --- Press 1 2");
	text.draw("   ");
	text.draw(" texture ID: " + stringify<int>(texnum) + "     --- Press +");
	text.draw("   ");
	text.draw(" normalizer: " + stringify<double>(normalizer) + "     --- Press 6 9");
	text.draw(" normalizer_u: " + stringify<double>(normalizer_u) + "     --- Press 4 5");
	text.draw(" normalizer_v: " + stringify<double>(normalizer_v) + "     --- Press 7 8");

	text.leaveTextInputMode();
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
}

void initGLVariables() {}

void drawStimulus() {
	drawVertices_triangles(texnum, my_vertices);
}

double calculateStimulus_ZfromY(double cylDepth, double vertexY) {
	double vertexZ;

	if (abs(vertexY) >= stimulus_height / 2) {
		vertexZ = 0;
	}
	else {
		vertexZ = cylDepth * sqrt(1 - pow(vertexY / (stimulus_height / 2.), 2));
	}
	return (vertexZ);
}

void buildVertices_triangles(double cylinderDepth, VerticesData& vertices_data) {
	vertices_data.vertices_vec.clear();
	vertices_data.colors_vec.clear();
	vertices_data.texcoors_vec.clear();
	vertices_data.indices_draw_triangle_vec.clear();

	double step_size_width = (stimulus_width / (nr_points_width - 1));
	double step_size_height = (stimulus_height / (nr_points_height - 1));

	int v_index = 0, t_index = 0, ind = 0; //using array 
	GLuint i_ind = 0;

	double x, y, z, y_prev, z_prev, u, v;
	y_prev = -stimulus_height / 2;
	z_prev = calculateStimulus_ZfromY(cylinderDepth, y_prev);
	double total_distance_y = 0; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse

	for (int j = 0; j < nr_points_height; j++) {  // 
		y = -stimulus_height / 2 + j * step_size_height;
		z = calculateStimulus_ZfromY(cylinderDepth, y);

		total_distance_y = total_distance_y + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));
		v = total_distance_y / normalizer_v; //v coordinate

		for (int i = 0; i < nr_points_width; i++) { //
			x = -stimulus_width / 2 + i * step_size_width;
			u = (x + stimulus_width / 2) / normalizer_u; //u coordinate. 

			/*
			step 1: build the meshgrid using vertices,


			(8)---(9)---(10)--(11)
			 |     |     |     |
			 |     |     |     |
			 |     |     |     |
			 |     |     |     |
			(4)---(5)---(6)---(7)
			 |     |     |     |
			 |     |     |     |
			 |     |     |     |
			 |     |     |     |
			(0)---(1)---(2)---(3)


			going over each vertex: store the (x,y,z) to vertices array or vector, (u,v) to texcoors array or vector, (1,0,0) to colors array or vector,
			if light source or shading is involved, normals are needed
			*/


			/*
			// uring array
			vertices[v_index] = x;
			colors[v_index] = 1;
			//normals[v_index] = 0;
			v_index++;
			vertices[v_index] = y;
			colors[v_index] = 0;
			//normals[v_index] = y * cylinderDepth * cylinderDepth;
			v_index++;
			vertices[v_index] = z;
			colors[v_index] = 0;
			//normals[v_index] = z * stimulus_height * stimulus_height /4;
			v_index++;

			texcoors[t_index] = u; //(x + stimulus_width / 2) / normalizer_u; //u coordinate.
			t_index++;
			texcoors[t_index] = v; //total_distance_y / normalizer_v; //v coordinate
			t_index++;
			*/


			// using vector
			vertices_data.vertices_vec.push_back(x);
			vertices_data.vertices_vec.push_back(y);
			vertices_data.vertices_vec.push_back(z);

			vertices_data.colors_vec.push_back(1);
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			vertices_data.texcoors_vec.push_back(u);
			vertices_data.texcoors_vec.push_back(v);

			/*
				step 2: create an array/vector that store how the triangles should be drawn

				The array/vector is one dimensional, but is groupped by unit of 3, meaning every three elements form a triangle

				for example, if indices_draw_triangle is like [0 1 4 4 1 5 1 2 5 5 2 6 ...],
				then it draws triangles with indices {0 1 4}, {4 1 5}, {1 2 5}, {5 2 6}...

				(8)---(9)---(10)--(11)
				 |\    |\    |\    |
				 | \   | \   | \   |
				 |  \  |  \  |  \  |
				 |   \ |   \ |   \ |
				(4)---(5)---(6)---(7)
				 |\    |\    |\    |
				 | \   | \   | \   |
				 |  \  |  \  |  \  |
				 |   \ |   \ |   \ |
				(0)---(1)---(2)---(3)

				triangle 1: 0 1 4;   triangle 2: 4 1 5
				triangle 3: 1 2 5;   triangle 2: 5 2 6 ...
			*/

			// construct the triangle indices to be drawn
			if (i < nr_points_width - 1 && j < nr_points_width - 1) {
				// using array
				/*
				indices_draw_triangle[ind] = i_ind; ind++;
				indices_draw_triangle[ind] = i_ind + 1; ind++;
				indices_draw_triangle[ind] = i_ind + nr_points_width; ind++;

				indices_draw_triangle[ind] = i_ind + nr_points_width; ind++;
				indices_draw_triangle[ind] = i_ind + 1; ind++;
				indices_draw_triangle[ind] = i_ind + nr_points_width + 1; ind++;
				*/

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);
				ind = ind + 6;
			}

			i_ind++;
		}

		y_prev = y; z_prev = z;
	}

	total_ind = ind;
}

void drawVertices_triangles(int texNum, const VerticesData& vertices_data) {
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	// enable matrices for use in drawing below
	//glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	//glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces

	// bind the texture
	glBindTexture(GL_TEXTURE_2D, loadedTexture[texNum]);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Use the two lines below (instead of the two lines above) to enable mipmapping for antialiasing
	// If you uncomment the two lines below, make sure to also uncomment the
	// "SOIL_FLAG_MIPMAPS" flag in LoadGLTextures() so that mipmaps are built.
	//glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	//glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	/*
		// using array
		glVertexPointer(3, GL_FLOAT, 0, vertices);
		glTexCoordPointer(2, GL_FLOAT, 0, texcoors);
		//glNormalPointer(GL_FLOAT, 0, normals); //
		glColorPointer(3, GL_FLOAT, 0, colors);
		glDrawElements(GL_TRIANGLES, total_ind, GL_UNSIGNED_INT, indices_draw_triangle);
	*/

	//using vector
	glVertexPointer(3, GL_FLOAT, 0, &vertices_data.vertices_vec[0]);
	glTexCoordPointer(2, GL_FLOAT, 0, &vertices_data.texcoors_vec[0]);
	//glNormalPointer(GL_FLOAT, 0, &vertices_data.normals_vec[0]); //
	glColorPointer(3, GL_FLOAT, 0, &vertices_data.colors_vec[0]);
	glDrawElements(GL_TRIANGLES, vertices_data.indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &vertices_data.indices_draw_triangle_vec[0]);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	//glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}


void initStimulus() {
	buildVertices_triangles(depth, my_vertices);
	initProjectionScreen(display_distance);
	texnum = 1;
}


void drawGLScene() {
	if (stereo) {
		//glDrawBuffer(GL_BACK);

		// Draw left eye view
		glDrawBuffer(GL_BACK_LEFT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		cam.setEye(eyeLeft);
		// cam.setEye(eyeRight); // we need to flip the eye because the image is mirrored
		drawStimulus();
		drawInfo();

		// Draw right eye view
		glDrawBuffer(GL_BACK_RIGHT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		cam.setEye(eyeRight);
		// cam.setEye(eyeLeft); // we need to flip the eye because the image is mirrored
		drawStimulus();
		drawInfo();

		glutSwapBuffers();
	} else {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		cam.setEye(eyeMiddle);
		drawStimulus();

		glutSwapBuffers();
	}
}

int LoadGLTextures() { // Load PNG And Convert To Textures
	for (int i = 0; i <= 4; i++) {
		std::stringstream ss;
		ss << i;

		string texturePath = "C:/cncsvision/experimentsbrown/ShapeConstancy/textures/test/test" + ss.str() + ".png";
		loadedTexture[i] = SOIL_load_OGL_texture
		(
			texturePath.c_str(),
			SOIL_LOAD_AUTO,
			SOIL_CREATE_NEW_ID,
			SOIL_FLAG_MULTIPLY_ALPHA // | SOIL_FLAG_MIPMAPS
		);
	}

	return true; // Return Success
}



void handleKeypress(unsigned char key, int x, int y)
{
	switch (key)
	{   //Quit program
		case 27:	//corrisponde al tasto ESC
		{
			if (resetScreen_betweenRuns)
				homeEverything(5000, 4500);
			exit(0);
		}
		break;

		case '1':
		{
			if (depth > depth_step) {
				depth = depth - depth_step;
			}


			buildVertices_triangles(depth, my_vertices);


		}
		break;

		case '2':
		{
			depth = depth + depth_step;
			buildVertices_triangles(depth, my_vertices);


		}
		break;

		case 'm':
		{
			interoculardistance--;
			eyeRight = Vector3d(interoculardistance / 2, 0, 0);
			eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
			break;
		}

		case 'n':
		{
			interoculardistance++;
			eyeRight = Vector3d(interoculardistance / 2, 0, 0);
			eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
			break;
		}

		case '+':
		{
			texnum = (texnum + 1) % 5;
			break;
		}

		case '4':
		{
			normalizer_u = normalizer_u - 10;
			buildVertices_triangles(depth, my_vertices);
			break;
		}

		case '5':
		{
			normalizer_u = normalizer_u + 10;
			buildVertices_triangles(depth, my_vertices);
			break;
		}

		case '7':
		{
			normalizer_v = normalizer_v - 10;
			buildVertices_triangles(depth, my_vertices);
			break;
		}

		case '8':
		{
			normalizer_v = normalizer_v + 10;
			buildVertices_triangles(depth, my_vertices);
			break;
		}


		case '6':
		{
			normalizer = normalizer - 10;
			normalizer_u = normalizer;
			normalizer_v = normalizer;
			buildVertices_triangles(depth, my_vertices);
			break;
		}

		case '9':
		{
			normalizer = normalizer + 10;
			normalizer_u = normalizer;
			normalizer_v = normalizer;
			buildVertices_triangles(depth, my_vertices);
			break;
		}
	}
}


void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}


void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE * SCREEN_HEIGHT / SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 3500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 3500);
}


void update(int value) {
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

void idle()
{}

void initRendering() {
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set depth buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set depth function */
	glDepthFunc(GL_LEQUAL);
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
	// ballerine (le vedi vibrare)
	glLineWidth(1.5);
}

void initVariables() {
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
	eyeMiddle = Vector3d(0, 0, 0);
	initStimulus();
}

// Inizializza gli stream, apre il file per poi scriverci
void initStreams() {}

// Porta tutti i motori nella posizione di home e azzera i contatori degli steps
void initMotors() {
	if (resetScreen_betweenRuns)
		homeEverything(5000, 4500);
}

int main(int argc, char* argv[])
{
	mathcommon::randomizeStart();
	glutInit(&argc, argv);
	if (stereo) {
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);
		//glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO | GLUT_MULTISAMPLE); // 1/1/2023: FreeGlut bug - if GLUT_MULTISAMPLE, NO Stereo 
	}
	else {
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	}

	if (gameMode == false)
	{
		glutInitWindowSize(640, 480);
		glutCreateWindow("EXP WEXLER");
		//glutFullScreen();
	}
	else
	{
		glutGameModeString("1024x768:32@85");
		glutEnterGameMode();
		glutFullScreen();
	}
	initMotors();
	initRendering();
	LoadGLTextures();
	initGLVariables();
	initStreams();
	initVariables();
	glutDisplayFunc(drawGLScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutIdleFunc(idle);
	glutTimerFunc(TIMER_MS, update, 0);
	glutSetCursor(GLUT_CURSOR_NONE);
	//boost::thread initVariablesThread(&initVariables);
	glutMainLoop();

	return 0;
}
