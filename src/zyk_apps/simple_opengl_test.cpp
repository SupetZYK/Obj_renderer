/* ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
//
// If you intend to _use_ this code sample in your app, do yourself a favour
// and replace immediate mode calls with VBOs ...
//
// The vc8 solution links against assimp-release-dll_win32 - be sure to
// have this configuration built.
// ----------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif

/* assimp include files. These three are usually needed. */
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
using namespace std;

/* the global Assimp scene object */
const  aiScene* scene = NULL;
GLuint scene_list = 0;
aiVector3D scene_min, scene_max, scene_center;
double radius=0;

int mainMenu, displayMenu; // glut menu handlers
int winWidth=500,winHeight=500;
double winAspect;
float scale_factor=1;
int lastX, lastY; // last mouse motion position
bool leftDown,leftUp, rightUp, rightDown, middleDown, middleUp, shiftDown; // mouse down and shift down flags
double sphi = 0.0, stheta = 0.0;// sdepth = 10; // for simple trackball
//double xpan = 0.0, ypan = 0.0; // for simple trackball
//double zNear = 1, zFar = 100.0;
//double g_fov = 45.0;
//double g_sdepth;
/* current rotation angle */
static float angle = 0.f;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

/* ---------------------------------------------------------------------------- */
//void reshape(int width, int height)
//{
//	const double aspectRatio = (float) width / height, fieldOfView = 45.0;

//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(fieldOfView, aspectRatio,
//		1.0, 1000.0);  /* Znear and Zfar */
//	glViewport(0, 0, width, height);
//}

/* ---------------------------------------------------------------------------- */
void get_bounding_box_for_node (const  aiNode* nd,
    aiVector3D* min,
    aiVector3D* max,
    aiMatrix4x4* trafo
){
    aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo,&nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
        const  aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		for (t = 0; t < mesh->mNumVertices; ++t) {

             aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp,trafo);

			min->x = aisgl_min(min->x,tmp.x);
			min->y = aisgl_min(min->y,tmp.y);
			min->z = aisgl_min(min->z,tmp.z);

			max->x = aisgl_max(max->x,tmp.x);
			max->y = aisgl_max(max->y,tmp.y);
			max->z = aisgl_max(max->z,tmp.z);
		}
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(nd->mChildren[n],min,max,trafo);
	}
	*trafo = prev;
}

/* ---------------------------------------------------------------------------- */
void get_bounding_box ( aiVector3D* min,  aiVector3D* max)
{
    aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	min->x = min->y = min->z =  1e10f;
	max->x = max->y = max->z = -1e10f;
	get_bounding_box_for_node(scene->mRootNode,min,max,&trafo);
}

/* ---------------------------------------------------------------------------- */
void color4_to_float4(const  aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

/* ---------------------------------------------------------------------------- */
void set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

/* ---------------------------------------------------------------------------- */
void apply_material(const aiMaterial *mtl)
{
    float c[4];

    GLenum fill_mode;
    int ret1, ret2;
     aiColor4D diffuse;
     aiColor4D specular;
     aiColor4D ambient;
     aiColor4D emission;
    float shininess, strength;
    int two_sided;
    int wireframe;
    unsigned int max;

    set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        color4_to_float4(&diffuse, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
        color4_to_float4(&specular, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

    set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
        color4_to_float4(&ambient, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
        color4_to_float4(&emission, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

    max = 1;
    ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
    if(ret1 == AI_SUCCESS) {
        max = 1;
        ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
        if(ret2 == AI_SUCCESS)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
    else {
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
        set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
    }

    max = 1;
    if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
        fill_mode = wireframe ? GL_LINE : GL_FILL;
    else
        fill_mode = GL_FILL;
    glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

    max = 1;
    if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
        glDisable(GL_CULL_FACE);
    else
        glEnable(GL_CULL_FACE);
}

/* ---------------------------------------------------------------------------- */
void recursive_render (const  aiScene *sc, const  aiNode* nd)
{
	unsigned int i;
	unsigned int n = 0, t;
     aiMatrix4x4 m = nd->mTransformation;

	/* update transform */
	aiTransposeMatrix4(&m);
	glPushMatrix();
	glMultMatrixf((float*)&m);

	/* draw all meshes assigned to this node */
	for (; n < nd->mNumMeshes; ++n) {
        const  aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
        apply_material(sc->mMaterials[mesh->mMaterialIndex]);
		if(mesh->mNormals == NULL) {
			glDisable(GL_LIGHTING);
		} else {
			glEnable(GL_LIGHTING);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
      const  aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;
			switch(face->mNumIndices) {
				case 1: face_mode = GL_POINTS; break;
				case 2: face_mode = GL_LINES; break;
				case 3: face_mode = GL_TRIANGLES; break;
				default: face_mode = GL_POLYGON; break;
			}

			glBegin(face_mode);
//      if(mesh->mNormals != NULL)
//        glNormal3fv(&mesh->mNormals[face->mIndices[2]].x);
      if(face_mode == GL_TRIANGLES && mesh->mNormals != NULL)
      {
        // USE FLAT SHADED
        aiVector3D & p1 = mesh->mVertices[face->mIndices[0]];
        aiVector3D & p2 = mesh->mVertices[face->mIndices[1]];
        aiVector3D & p3 = mesh->mVertices[face->mIndices[2]];
        float x1=p3[0]-p1[0];
        float y1=p3[1]-p1[1];
        float z1=p3[2]-p1[2];
        float x2=p2[0]-p1[0];
        float y2=p2[1]-p1[1];
        float z2=p2[2]-p1[2];

        aiVector3D normal(y1*z2-y2*z1,x2*z1-x1*z2,x1*y2-x2*y1);
        normal= -normal.Normalize();
        glNormal3fv(&normal.x);
      }
			for(i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
        if(mesh->mColors[0] != NULL)
          glColor4fv((GLfloat*)&mesh->mColors[0][index]);
        if(face_mode == GL_POLYGON && mesh->mNormals != NULL)
        {
          //USE SMOOTH SHADED
          glNormal3fv(&mesh->mNormals[index].x);
        }
        glVertex3fv(&mesh->mVertices[index].x);
			}

			glEnd();
		}

	}

	/* draw all children */
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursive_render(sc, nd->mChildren[n]);
	}

	glPopMatrix();
}

/* ---------------------------------------------------------------------------- */
//void do_motion (void)
//{
//	static GLint prev_time = 0;
//	static GLint prev_fps_time = 0;
//	static int frames = 0;

//	int time = glutGet(GLUT_ELAPSED_TIME);
//	angle += (time-prev_time)*0.01;
//	prev_time = time;

//	frames += 1;
//	if ((time - prev_fps_time) > 1000) /* update every seconds */
//    {
//        int current_fps = frames * 1000 / (time - prev_fps_time);
//        printf("%d fps\n", current_fps);
//        frames = 0;
//        prev_fps_time = time;
//    }


//	glutPostRedisplay ();
//}

/* ---------------------------------------------------------------------------- */
// GLUT display callback function
void DisplayFunc() {
	float tmp;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
  gluLookAt(0.f,0.f,3,0.f,0.f,0.f,0.f,1.f,0.f);
//        GLfloat LightPosition[]= { 0.0f, 0.0f, -1.0f, 0.0f };
//        glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
//        glEnable(GL_LIGHT0);
//    glTranslatef(xpan, ypan, -sdepth);
	/* rotate it around the y axis */
    //glRotatef(angle,0.f,1.f,0.f);
    glRotatef(-stheta, 1.0, 0.0, 0.0);
    glRotatef(sphi, 0.0, 1.0, 0.0);

	/* scale the whole asset to fit into our view frustum */
    tmp = scene_max.x-scene_min.x;
    tmp = aisgl_max(scene_max.y - scene_min.y,tmp);
    tmp = aisgl_max(scene_max.z - scene_min.z,tmp);
    tmp = scale_factor / tmp;
    glScalef(tmp, tmp, tmp);

        /* center the model */
	glTranslatef( -scene_center.x, -scene_center.y, -scene_center.z );

        /* if the display list has not been made yet, create a new one and
           fill it with scene contents */
	if(scene_list == 0) {
	    scene_list = glGenLists(1);
	    glNewList(scene_list, GL_COMPILE);
            /* now begin at the root node of the imported data and traverse
               the scenegraph by multiplying subsequent local transforms
               together on GL's matrix stack. */
	    recursive_render(scene, scene->mRootNode);
	    glEndList();
	}

	glCallList(scene_list);

	glutSwapBuffers();

}

// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y) {
    switch (ch) {
    case '6':
        cout<<"0006"<<endl;
        break;
    case '9':
        cout<<"0009"<<endl;
        break;
    case 27:
        exit(0);
        break;
    }
    glutPostRedisplay();
}



// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {

    lastX = x;
    lastY = y;
    leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
    leftUp = (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP);
    rightDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
    rightUp = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_UP);
    middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
    middleUp = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_UP);
    shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);
}

// GLUT mouse motion callback function
void MotionFunc(int x, int y) {
    if (leftDown)
        if(!shiftDown) { // rotate
            sphi += (double)(x - lastX) / 4.0;
            stheta += (double)(lastY - y) / 4.0;
        } /*else { // pan
            xpan += (double)(x - lastX)*sdepth/zNear/winWidth;
            ypan += (double)(lastY - y)*sdepth/zNear/winHeight;
        }*/
    // scale
    if (middleDown)
        scale_factor*=(double)(1+(lastY-y)/20.0);

    lastX = x;
    lastY = y;
    glutPostRedisplay();
}

/* ---------------------------------------------------------------------------- */
int loadasset (const char* path)
{
	/* we are taking one of the postprocessing presets to avoid
	   spelling out 20+ single postprocessing flags here. */
	scene = aiImportFile(path,aiProcessPreset_TargetRealtime_MaxQuality);

	if (scene) {
		get_bounding_box(&scene_min,&scene_max);
		scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
		scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
		scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

        double PI = 3.14159265358979323846;
        radius=(scene_max-scene_min).Length()/2;
		return 0;
	}
	return 1;
}





// GLUT menu callback function
void MenuCallback(int value) {
    switch (value) {
    case 99: exit(0); break;
    default:
        glutPostRedisplay();
        break;
    }
}

// init right-click menu
void InitMenu() {
    displayMenu = glutCreateMenu(MenuCallback);
    mainMenu = glutCreateMenu(MenuCallback);
    glutAddMenuEntry("Exit", 99);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

// GLUT reshape callback function
void ReshapeFunc(int width, int height) {
    const double aspectRatio = (float) width / height, fieldOfView = 45.0;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfView, aspectRatio,
        1.0, 1000.0);  /* Znear and Zfar */
    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

// init openGL environment
void InitGL() {
    /**basic setting**/
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(winWidth, winHeight);
    glutInitWindowPosition(100,100);
    glutCreateWindow("Mesh Viewer");
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glPolygonOffset(1.0, 1.0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);  // try this!
    glEnable(GL_NORMALIZE);

    /* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
    if(getenv("MODEL_IS_BROKEN"))
        glFrontFace(GL_CW);
    /**light setting**/
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);    /* Uses default lighting parameters */
//    GLfloat LightAmbient[]= { 0.4f, 0.4f, 0.4f, 1.0f };
//    GLfloat LightDiffuse[]= { 0.6f, 0.6f, 0.6f, 1.0f };
//    GLfloat LightPosition[]= { 0.0f, 0.0f, -1000.0f, 0.0f };
//    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
//    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
//    glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
//    glEnable(GL_LIGHT1);
    /**material setting**/
//    glLightModeli(GL_LIGHT_MODEL_AMBIENT, GL_TRUE);
//    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,LightAmbient);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    /**Others**/
    glutReshapeFunc(ReshapeFunc);
    glutDisplayFunc(DisplayFunc);
    glutKeyboardFunc(KeyboardFunc);
    glutMouseFunc(MouseFunc);
    glutMotionFunc(MotionFunc);
}
/* ---------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
     aiLogStream stream;

    glutInit(&argc, argv);
    InitGL();
    InitMenu();
//	glutDisplayFunc(display);
//	glutReshapeFunc(reshape);

	/* get a handle to the predefined STDOUT log stream and attach
	   it to the logging system. It remains active for all further
	   calls to aiImportFile(Ex) and aiApplyPostProcessing. */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
	aiAttachLogStream(&stream);

	/* ... same procedure, but this stream now writes the
	   log messages to assimp_log.txt */
	stream = aiGetPredefinedLogStream(aiDefaultLogStream_FILE,"assimp_log.txt");
	aiAttachLogStream(&stream);

	/* the model name can be specified on the command line. If none
	  is specified, we try to locate one of the more expressive test
	  models from the repository (/models-nonbsd may be missing in
	  some distributions so we need a fallback from /models!). */
	if( 0 != loadasset( argc >= 2 ? argv[1] : "../../test/models-nonbsd/X/dwarf.x")) {
		if( argc != 1 || (0 != loadasset( "../../../../test/models-nonbsd/X/dwarf.x") && 0 != loadasset( "../../test/models/X/Testwuson.X"))) {
			return -1;
		}
	}



	glutGet(GLUT_ELAPSED_TIME);
	glutMainLoop();

	/* cleanup - calling 'aiReleaseImport' is important, as the library
	   keeps internal resources until the scene is freed again. Not
	   doing so can cause severe resource leaking. */
	aiReleaseImport(scene);

	/* We added a log stream to the library, it's our job to disable it
	   again. This will definitely release the last resources allocated
	   by Assimp.*/
	aiDetachAllLogStreams();
	return 0;
}
