#pragma region Region_Library
/*--------------Standard-------------*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <iomanip>
/*--------------OpenCV-------------*/
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv.h>
/*--------------Kinect-------------*/
#include <Kinect.h>
#include <windows.h>   
/*--------------OpenGL-------------*/
#include <gl/Gl.h>
#include <gl/glu.h>
#include <gl/glut.h>
//将库文件链接到文件中
#pragma comment(lib,"opengl32.lib")
#pragma comment(lib,"GLU32.LIB")
#pragma comment(lib,"GLUT32.LIB")
/*-------------LoadOBJ-------------*/
#include "glm.h"
/*------------PortAduio------------*/
#include "portaudio.h"
using namespace std;
using namespace cv;
/*--------------M232---------------*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <iomanip>
#include "dllmain.h"
#using <System.dll>
using namespace System;
using namespace System::IO::Ports;
using namespace System::ComponentModel;
using namespace System::Threading;

#pragma endregion Region_Library

#pragma region Global_variable

#pragma region OpenCV&ROI
/*---------------------------------*/
/*--------------OpenCV-------------*/
/*---------------------------------*/
Rect2i ROI_rect;
Point ROI_p1, ROI_p2;
Mat ROI;
bool ROI_S1 = FALSE;
bool ROI_S2 = FALSE;
void onMouseROI(int event, int x, int y, int flags, void* param);
//DataType: 0 = unsigned char, 1 = float 这里是读取mat的数据
void InputValue(Mat M, int DataType, int Row, int Col, int Chan, float Data);
void OutputValue(Mat M, int Row, int Col, int Chan, uchar* Data);
void OutputValue(Mat M, int Row, int Col, int Chan, float* Data);
void FindROI(void);									//Find ROI Func
//Tracking
int ROIcount = 0;
Point2i ROICenterColorS_Old, ROICenterColorS_New;   //center of tracking object of ROI in Color Space
Point2i* ROIPixel = nullptr;
CameraSpacePoint* ROICameraSP = nullptr;
void MoveROI(void);                                //Tracking ROI Func
//Mapping
int ROIDepthCount = 0;
Point3f ROICenterCameraS;
void CameraSpaceROI(void);                         //Mapping ROI Func
//Storage
CameraSpacePoint* ROICameraSP_Storage = nullptr;					//Probe tip on GL Coordinate
CameraSpacePoint* ROICameraSP_Proj_Storage = nullptr;				//Probe tip projection on GL Coordinate
CameraSpacePoint* ROICameraSP_MachineCoord_Storage = nullptr;			//Probe tip on Machine Coordinate
CameraSpacePoint* ROICameraSP_MachineCoord_Proj_Storage = nullptr;		//Probe tip projection on Machine Coordinate
const int StorageSize = 5000;						//How many points do you want to record
int ROIStorageCount = 0;							//Total # of points stored
bool IS_KEY_F1_UP = FALSE;							//FLAG of data collecting, see SpecialKeys()
void ROICameraSPStorage(void);						//Storage ROI Func
CameraSpacePoint* ROICameraSP_TouchDetec = nullptr;
CameraSpacePoint* ROICameraSP_MechCoord = nullptr;
CameraSpacePoint* ROICameraSP_Proj_MechCoord = nullptr;
void ROITrans(CameraSpacePoint* Data, int DataNum, GLfloat* TransM, CameraSpacePoint* Result);
void ROITrans(float* Data, int DataNum, GLfloat* TransM, float* Result);
#pragma endregion OpenCV&ROI

#pragma region Kinect
/*---------------------------------*/
/*--------------Kinect-------------*/
/*---------------------------------*/
//Initial (Sensor)
IKinectSensor* pSensor = nullptr;
ICoordinateMapper* Mapper = nullptr;
IDepthFrameReader* pFrameReaderDepth = nullptr;
IColorFrameReader* pFrameReaderColor = nullptr;
void KinectInit(void);					//Kinect Initial
void KinectUpdate(void);				//Kinect Update Frame
//Depth Map
int iWidthDepth = 0;
int iHeightDepth = 0;
UINT depthPointCount = 0;
UINT16* pBufferDepth = nullptr;			//Depth map origin format
UINT16 uDepthMin = 0, uDepthMax = 0;
Mat mDepthImg;							//Depth map UINT16
Mat dstDepthImg;
Mat mImg8bit;							//Depth map CV_8U
//Color Map
int iWidthColor = 0;
int iHeightColor = 0;
UINT colorPointCount = 0;
UINT uBufferSizeColor = 0;
BYTE* pBufferColor = nullptr;			//Color map origin format (RGBA)
Mat mColorImg;							//Color map OpenCV format (BGRA)
Mat dstImage;
//Map to camera space point
CameraSpacePoint* pCSPoints = nullptr;
void ShowImage(void);					//Display Image Func
#pragma endregion Kinect

#pragma region OpenGL&DrawCubic
/*---------------------------------*/
/*--------------OpenGL-------------*/
/*---------------------------------*/
//Basic OpenGL Function
void GLInit(void);
void SpecialKeys(int key, int x, int y);
void Keyboard(unsigned char key, int x, int y);
void timer(int value);					//OpenGL Time Func
void RenderScene(void);					//OpenGL Render Func
bool BG_IS_OPEN = TRUE;					//Flag for BG
void SceneWithBackground(void);			//Background Func
void SceneWithoutBackground(void);
void DrawPointCloud(void);				//Drawing Type
void DrawMeshes(void);
void Draw3DLine(void);
void Draw3DPlane(void);
int FPS = 0;
GLuint textureid;						//BG texture
//Calculating FPS w/o waiting monitor to display
int FPS_RS = 0;
bool Finish_Without_Update = FALSE;
float g_fps(void(*func)(void), int n_frame);
//Draw Cubic
Point3f CubicPosi;						//Manually translate the .obj
Point3f CubicRota;						//Manually rotate the .obj (seldom use)
Point3f ObjPosi;						//.obj translation
Point3f TipPosi;						//Prob tip translation
bool ROI_IS_COLLIDE = FALSE;			//FLAG of if prob insert into the virtual cube
bool Cubic_IS_BLEND = FALSE;
bool CUBIC_MOVE = FALSE;				//FLAG to translate the .obj while plate moves
bool ARFunc_IS_ON = TRUE;				//FLAG to show AR Function or nor
float M_Cubic[16] = {					//Machine Coordinate rotation
	-0.9998806737707267, 0.005917352641148399, -0.01426965863354962, 0,
	-0.003275900585287705, 0.8214974199757811, 0.570202821326316, 0,
	0.01509657892216168, 0.5701815271567955, -0.8213800091273172, 0,
	0, 0, 0, 1 };
float M_Cubic_inv[16] = {				//Machine Coordinate rotation inversion
	-0.9998806737707267, -0.003275900585287447, 0.01509657892216168, 0,
	0.005917352641148182, 0.821497419975781, 0.5701815271567955, 0,
	-0.01426965863354976, 0.5702028213263157, -0.8213800091273169, 0,
	0, 0, 0, 1 };
CameraSpacePoint Intersect;				//Machine Coordinate translation
void DrawCubic(void);
//Creat Pop Up menu
enum {
	MENU_WTBG_NONE = 1,
	MENU_WTBG_MESH,
	MENU_3D_LINE,
	MENU_3D_PLANE,
	MENU_WTBG_POINTCLOUD,
	MENU_WOBG_MESH,
	MENU_WOBG_POINTCLOUD,
	MENU_STORAGETYPE1,
	MENU_STORAGETYPE2,

};
int menu_value = 0;
int submenuID_BG;
int submenuID_WTBG_DrawType;
int submenuID_WOBG_DrawType;
int submenuID_StorageType;
int DRAWING_TYPE = 0;                      //Flag for drawing type: 0 = none, 1 = mesh, 2 = point cloud
int STORAGE_TYPE = 0;                      //Flag for storage type: 0 = prob tip pt, 1 = projection pt
void BuildPopupMenu(void);
void menuCB(int menu_value);
/*what does this region mean? */
//glPrint
DWORD fdwCharSet = CHINESEBIG5_CHARSET;		// Encoding code: Big Five
char szFace[] = "Courier New";				// Script: Courier New
int nHeight = 16;							// Character Size: 16 pixel
//记录字符的字型图像是否已被加载, 如果某字符的字型图像已被加载, 则不需再载入
GLuint base = 0;							// 字型图像 的 display lists, 初始成 0
//窗口系统的东西, 我们要透过它才能向 窗口系统 取得 某字符的字型图像
bool loaded[65536] = { 0 };					// 定义 65536 个 bool, 全都初始成 0
HFONT hFont;								// font handle
int glPrintf(const char *format, ...);
#pragma endregion OpenGL&DrawCubic

#pragma region LoadOBJ
/*---------------------------------*/
/*-------------LoadOBJ-------------*/
/*---------------------------------*/
GLMmodel* OBJ;
GLuint textures[6];
#define MAX_TEXTURE_NUM 50
#define MAXSIZE 200000
#define MAXGROUPSIZE 50
#define myMax(a,b) (((a)>(b))?(a):(b))
float vertices[MAXGROUPSIZE][MAXSIZE];
float normals[MAXGROUPSIZE][MAXSIZE];
float vtextures[MAXGROUPSIZE][MAXSIZE];
void loadOBJModel(void);
void SetTexObj(char *name, int i);
void Texture(void);
void traverseModel(void);
#pragma endregion LoadOBJ

#pragma region M232&Dispenser
/*---------------------------------*/
/*--------------M232---------------*/
/*---------------------------------*/
bool MtHome(void);
void MtReflashWait(void);
void MtInit(void);
void MtMove(void);
void MtCheck(void);
//To adjust error due to calibration while .obj moves along the plates on y axis
float dev_theta = 5 * CV_PI / 180;
CameraSpacePoint* DeviaDueToY = nullptr;
//To synchronize machine and .obj movement
int mtmove_step = 0;
Rs232MotionData* md = new struct Rs232MotionData;	//Tracking machine status
#pragma endregion M232&Dispenser

#pragma region AR Function
CameraSpacePoint* ARFunc_ROICSP_Proj = nullptr;		//Projection point of the probe tip
double ARFuncNormal[4] = { 0 };						//Top plane planner function of the .OBJ file
float ARFunc_ROICSP_Proj_Dist = 0;					//Distance between probe tip and its projection point
void ARFunc_FindProj(void);	//Find projection point
//Check if the point is inside triangle
bool ARFunc_InsideTriCheck(CameraSpacePoint* Point, CameraSpacePoint* TriVertex0, CameraSpacePoint* TriVertex1, CameraSpacePoint* TriVertex2);
float Dot(float* Vector1, float* Vector2);			//Dot product
//|a b|
//|c d|
float Determinant(float a, float b, float c, float d);
bool IS_ARFunc_InsideTriCheck = FALSE;
bool IS_AR_PLANE_FUNC_FIND = FALSE;
#pragma endregion AR Function

#pragma region PortAudio
/*---------------------------------*/
/*------------PortAduio------------*/
/*---------------------------------*/
#define NUM_SECONDS   (10)
#define SAMPLE_RATE   (44100)
#define FRAMES_PER_BUFFER  (64)

#define TABLE_SIZE   (200)
/*What is the part?*/
class Sine
{
public:
	Sine() : stream(0), left_phase(0), right_phase(0)
	{
		/* initialise sinusoidal wavetable */
		for (int i = 0; i<TABLE_SIZE; i++)
		{
			sine[i] = (float)sin(((double)i / (double)TABLE_SIZE) * CV_PI * 2.);
		}
		sprintf(message, "No Message");
	}
	bool open(PaDeviceIndex index)
	{
		PaStreamParameters outputParameters;
		outputParameters.device = index;
		if (outputParameters.device == paNoDevice) {
			return false;
		}
		const PaDeviceInfo* pInfo = Pa_GetDeviceInfo(index);
		if (pInfo != 0)
		{
			printf("Output device name: '%s'\r", pInfo->name);
		}
		outputParameters.channelCount = 2;       /* stereo output */
		outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
		outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
		outputParameters.hostApiSpecificStreamInfo = NULL;
		PaError err = Pa_OpenStream(
			&stream,
			NULL, /* no input */
			&outputParameters,
			SAMPLE_RATE,
			paFramesPerBufferUnspecified,
			paClipOff,      /* we won't output out of range samples so don't bother clipping them */
			&Sine::paCallback,
			this            /* Using 'this' for userData so we can cast to Sine* in paCallback method */
			);
		if (err != paNoError)
		{
			/* Failed to open stream to device !!! */
			return false;
		}
		err = Pa_SetStreamFinishedCallback(stream, &Sine::paStreamFinished);
		if (err != paNoError)
		{
			Pa_CloseStream(stream);
			stream = 0;
			return false;
		}
		return true;
	}
	bool close()
	{
		if (stream == 0)
			return false;
		PaError err = Pa_CloseStream(stream);
		stream = 0;
		return (err == paNoError);
	}
	bool start()
	{
		if (stream == 0)
			return false;
		PaError err = Pa_StartStream(stream);
		return (err == paNoError);
	}
	bool stop()
	{
		if (stream == 0)
			return false;
		PaError err = Pa_StopStream(stream);
		return (err == paNoError);
	}
private:
	/* The instance callback, where we have access to every method/variable in object of class Sine */
	int paCallbackMethod(const void *inputBuffer, void *outputBuffer,
		unsigned long framesPerBuffer,
		const PaStreamCallbackTimeInfo* timeInfo,
		PaStreamCallbackFlags statusFlags)
	{
		float *out = (float*)outputBuffer;
		unsigned long i;
		(void)timeInfo; /* Prevent unused variable warnings. */
		(void)statusFlags;
		(void)inputBuffer;
		int key = 0;
		float distant = ARFunc_ROICSP_Proj_Dist * 1000.0f;
		//if (ARFunc_ROICSP_Proj_Dist * 1000 > 100)
		//{
		//	key = 5;
		//}
		//else if (ARFunc_ROICSP_Proj_Dist * 1000 > 50)
		//{
		//	key = 9;
		//}
		//else if (ARFunc_ROICSP_Proj_Dist * 1000 > 10)
		//{
		//	key = 13;
		//}
		//else
		//{
		//	key = 15;
		//}
		const int low_range = 10, high_range = 100, low_tone = 3, high_tone = 15;
		if (distant > high_range)
		{
			key = low_tone;
		}
		else if (distant < low_range)
		{
			key = high_tone;
		}
		else
		{
			key = (distant - low_range) / (high_range - low_range) * (low_tone - high_tone) + high_tone;
		}
		for (i = 0; i<framesPerBuffer; i++)
		{
			*out++ = sine[left_phase];  /* left */
			*out++ = sine[right_phase];  /* right */
			//cout << *out << endl;
			//left_phase += 10;
			left_phase += key;
			if (left_phase >= TABLE_SIZE) left_phase -= TABLE_SIZE;
			//right_phase += 3; /* higher pitch so we can distinguish left and right. */
			right_phase += key;
			if (right_phase >= TABLE_SIZE) right_phase -= TABLE_SIZE;
			//cout << key << endl;
		}
		return paContinue;
	}
	/* This routine will be called by the PortAudio engine when audio is needed.
	** It may called at interrupt level on some machines so don't do anything
	** that could mess up the system like calling malloc() or free().
	*/
	static int paCallback(const void *inputBuffer, void *outputBuffer,
		unsigned long framesPerBuffer,
		const PaStreamCallbackTimeInfo* timeInfo,
		PaStreamCallbackFlags statusFlags,
		void *userData)
	{
		/* Here we cast userData to Sine* type so we can call the instance method paCallbackMethod, we can do that since
		we called Pa_OpenStream with 'this' for userData */
		return ((Sine*)userData)->paCallbackMethod(inputBuffer, outputBuffer,
			framesPerBuffer,
			timeInfo,
			statusFlags);
	}
	void paStreamFinishedMethod()
	{
		printf("Stream Completed: %s\n", message);
	}
	/*
	* This routine is called by PortAudio when playback is done.
	*/
	static void paStreamFinished(void* userData)
	{
		return ((Sine*)userData)->paStreamFinishedMethod();
	}
	PaStream *stream;
	float sine[TABLE_SIZE];
	int left_phase;
	int right_phase;
	char message[20];
};
/*What's this part?*/
class ScopedPaHandler
{
public:
	ScopedPaHandler()
		: _result(Pa_Initialize())
	{
	}
	~ScopedPaHandler()
	{
		if (_result == paNoError)
		{
			Pa_Terminate();
		}
	}
	PaError result() const { return _result; }
private:
	PaError _result;
};
Sine sine;
ScopedPaHandler paInit;
bool InitPortAudio(void);
bool IS_PORTAUDIO_INIT = FALSE;
bool IS_PORTAUDIO_START = FALSE;
#pragma endregion PortAudio

#pragma endregion Global_variable

#pragma region Function

#pragma region OpenCV&ROI Function
void ROITrans(CameraSpacePoint* Data, int DataNum, GLfloat* TransM, CameraSpacePoint* Result)
{
	for (int i = 0; i < DataNum; i++)
	{
		GLfloat m[16] = { 0 };
		m[3] = m[7] = m[11] = m[15] = 1;
		m[12] = Data[i].X;
		m[13] = Data[i].Y;
		m[14] = Data[i].Z;
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(TransM);
		glMultMatrixf(m);
		glGetFloatv(GL_MODELVIEW_MATRIX, m);
		glPopMatrix();
		Result[i].X = m[12] / m[15];
		Result[i].Y = m[13] / m[15];
		Result[i].Z = m[14] / m[15];
	}
}

void ROITrans(float* Data, int DataNum, GLfloat* TransM, float* Result)
{
	for (int i = 0; i < DataNum; i++)
	{
		GLfloat m[16] = { 0 };
		m[0] = m[5] = m[10] = m[15] = 1;
		m[12] = Data[3 * i + 0];
		m[13] = Data[3 * i + 1];
		m[14] = Data[3 * i + 2];
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(TransM);
		glMultMatrixf(m);
		glGetFloatv(GL_MODELVIEW_MATRIX, m);
		glPopMatrix();
		Result[3 * i + 0] = m[12] / m[15];
		Result[3 * i + 1] = m[13] / m[15];
		Result[3 * i + 2] = m[14] / m[15];
	}
}

void InputValue(Mat M, int DataType, int Row, int Col, int Chan, float Data)
{
	int Steps = M.cols*M.channels();
	int Channels = M.channels();
	if (DataType == 0)
	{
		uchar* srcData = M.data;
		*(srcData + Row*Steps + Col*Channels + Chan) = Data;
	}
	else if (DataType == 1)
	{
		float* srcData = (float*)M.data;
		*(srcData + Row*Steps + Col*Channels + Chan) = Data;
	}
}

void OutputValue(Mat M, int Row, int Col, int Chan, uchar* Data)
{
	int Steps = M.cols*M.channels();
	int Channels = M.channels();
	uchar* srcData = M.data;
	*Data = *(srcData + Row*Steps + Col*Channels + Chan);
}

void OutputValue(Mat M, int Row, int Col, int Chan, float* Data)
{
	int Steps = M.cols*M.channels();
	int Channels = M.channels();
	float* srcData = (float*)M.data;
	*Data = *(srcData + Row*Steps + Col*Channels + Chan);
}
#pragma endregion OpenCV&ROI Function

#pragma region OpenGL Function
void GLInit()
{
	CubicPosi.x = 0;
	CubicPosi.y = 0;
	CubicPosi.z = 0;

	Intersect.X = 0.3610493931202467;
	Intersect.Y = -0.2131262817002425;
	Intersect.Z = -1;

	ObjPosi.x = -0.266 - 0.003;
	ObjPosi.y = 0.067 + 0.099;
	ObjPosi.z = 0.415 - 0.089;

	TipPosi.x = 0;
	TipPosi.y = -0.035;
	TipPosi.z = -0.12;

	DeviaDueToY = new CameraSpacePoint[1];
	DeviaDueToY->X = DeviaDueToY->Y = DeviaDueToY->Z = 0;
	GLfloat  whiteLight[] = { 0.45f, 0.45f, 0.45f, 1.0f };
	GLfloat  sourceLight[] = { 0.25f, 0.25f, 0.25f, 1.0f };
	GLfloat	 lightPos[] = { -50.f, 25.0f, 250.0f, 0.0f };

	glEnable(GL_DEPTH_TEST);
	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, whiteLight);
	glLightfv(GL_LIGHT0, GL_AMBIENT, sourceLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sourceLight);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glGenTextures(1, &textureid);
	glBindTexture(GL_TEXTURE_2D, textureid);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, iWidthColor, iHeightColor, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBufferColor);
}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'F':
	case 'f':
		/*Use CPU to get the FPS of the machine*/
		Finish_Without_Update = TRUE;
		//printf("%f fps\n", g_fps(RenderScene, 100));
		//cout << "FPS_RS = " << FPS_RS << endl;
		Finish_Without_Update = FALSE;
		break;
		/*A move */
	case 'A':
	case 'a':
		MtReflash(md);
		MtMove();
		CUBIC_MOVE = TRUE;
		ARFunc_IS_ON = FALSE;
		break;
	case 'S':
	case 's':
		ROICameraSPStorage();
		break;
	case 'H':
	case 'h':
		long MtEmpty();
		bool IS_HOME = MtHome();
		break;
	}
}

float g_fps(void(*func)(void), int n_frame)
{
	clock_t start, finish;
	int i;
	float fps;
	printf("Performing benchmark, please wait");
	start = clock();
	for (i = 0; i < n_frame; i++)
	{
		func();
	}
	printf("done\n");
	finish = clock();
	fps = float(n_frame) / (finish - start)*CLOCKS_PER_SEC;
	return fps;
}

void timer(int value)
{
	switch (value)
	{
		//OpenGL refresh timer
	case 0:
		printf("FPS = %d\n", FPS);
		FPS = 0;
		FPS_RS = 0;
		glutTimerFunc(1000, timer, 0);
		break;
	}
}

/*This part mean what?*/
void BuildPopupMenu()
{
	//3rd layer
	submenuID_WTBG_DrawType = glutCreateMenu(menuCB);
	glutAddMenuEntry("None", MENU_WTBG_NONE);
	glutAddMenuEntry("Mesh", MENU_WTBG_MESH);
	glutAddMenuEntry("Point Cloud", MENU_WTBG_POINTCLOUD);
	glutAddMenuEntry("3D Line Dection", MENU_3D_LINE);
	glutAddMenuEntry("3D Plane Dection", MENU_3D_PLANE);
	submenuID_WOBG_DrawType = glutCreateMenu(menuCB);
	glutAddMenuEntry("Mesh", MENU_WOBG_MESH);
	glutAddMenuEntry("Point Cloud", MENU_WOBG_POINTCLOUD);
	//2nd layer
	submenuID_BG = glutCreateMenu(menuCB);
	glutAddSubMenu("Color Image Background", submenuID_WTBG_DrawType);
	glutAddSubMenu("Empty Background", submenuID_WOBG_DrawType);
	submenuID_StorageType = glutCreateMenu(menuCB);
	glutAddMenuEntry("ROI Center Point", MENU_STORAGETYPE1);
	glutAddMenuEntry("Projection Point", MENU_STORAGETYPE2);
	//1st layer
	glutCreateMenu(menuCB);
	glutAddSubMenu("Background type", submenuID_BG);
	glutAddSubMenu("Storage type", submenuID_StorageType);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void menuCB(int menu_value)
{
	cout << menu_value << endl;
	switch (menu_value)	{
	case MENU_WTBG_NONE:
		BG_IS_OPEN = TRUE;
		DRAWING_TYPE = 0;
		break;
	case MENU_WTBG_MESH:
		BG_IS_OPEN = TRUE;
		DRAWING_TYPE = 1;
		break;
	case MENU_WTBG_POINTCLOUD:
		BG_IS_OPEN = TRUE;
		DRAWING_TYPE = 2;
		break;
	case MENU_3D_LINE:
		BG_IS_OPEN = TRUE;
		DRAWING_TYPE = 3;
	case MENU_3D_PLANE:
		BG_IS_OPEN = TRUE;
		DRAWING_TYPE = 4;
	case MENU_WOBG_MESH:
		BG_IS_OPEN = FALSE;
		DRAWING_TYPE = 1;
		break;
	case MENU_WOBG_POINTCLOUD:
		BG_IS_OPEN = FALSE;
		DRAWING_TYPE = 2;
		break;
	case MENU_STORAGETYPE1:
		STORAGE_TYPE = 0;
		break;
	case MENU_STORAGETYPE2:
		STORAGE_TYPE = 1;
		break;
	}
}

void SceneWithBackground()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, iWidthColor, iHeightColor);
	glOrtho(0, iWidthColor, 0, iHeightColor, -1, 1);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//Draw Background through RGBA image from Kinect
	glRasterPos2f(0.0f, 0.0f);
	//glDrawPixels(iWidthColor, iHeightColor, GL_RGBA, GL_UNSIGNED_BYTE, pBufferColor);
	glEnable(GL_TEXTURE_2D);
	//Without painting BG white the BG will turn orange and glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) did not work. Don't know why
	glColor3ub(255, 255, 255);
	// tell OpenGL to use the generated texture name
	glBindTexture(GL_TEXTURE_2D, textureid);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, iWidthColor, iHeightColor, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBufferColor);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	bool MirrorFunction = FALSE;
	if (!MirrorFunction)
	{
		glPushMatrix();
		glBegin(GL_QUADS);
		glTexCoord2i(0, 0);
		glVertex2i(0, iHeightColor);
		glTexCoord2i(1, 0);
		glVertex2i(iWidthColor, iHeightColor);
		glTexCoord2i(1, 1);
		glVertex2i(iWidthColor, 0);
		glTexCoord2i(0, 1);
		glVertex2i(0, 0);
		glEnd();
		glPopMatrix();
	}
	else{
		glPushMatrix();
		glBegin(GL_QUADS);
		glTexCoord2i(1, 0);
		glVertex2i(0, iHeightColor);
		glTexCoord2i(0, 0);
		glVertex2i(iWidthColor, iHeightColor);
		glTexCoord2i(0, 0);
		glVertex2i(iWidthColor, 0);
		glTexCoord2i(1, 1);
		glVertex2i(0, 0);
		glEnd();
		glPopMatrix();
	}
	glDisable(GL_TEXTURE_2D);
	static const double kFovY = 53.3;
	//static const double kPI = 3.1415926535897932384626433832795;
	double nearDist, farDist, aspect;
	nearDist = 0.01f / tan((kFovY / 2.0) * CV_PI / 180.0);
	farDist = 20000;
	aspect = (double)iWidthColor / iHeightColor;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(kFovY, aspect, nearDist, farDist);
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, -1, 0, 1, 0);
	glTranslatef(0.055f, -0.015f, 0.0f);
}

void SceneWithoutBackground()
{
	static const double kFovY = 53.3;
	//static const double kPI = 3.1415926535897932384626433832795;
	double nearDist, farDist, aspect;
	nearDist = 0.01f / tan((kFovY / 2.0) * CV_PI / 180.0);
	farDist = 20000;
	aspect = (double)iWidthColor / iHeightColor;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(kFovY, aspect, nearDist, farDist);
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, -1, 0, 1, 0);
	glTranslatef(0.055f, -0.015f, 0.0f);
}

void DrawPointCloud()
{
	/**************/
	/*************/
	glPushMatrix();
	glPointSize(1.0f);
	glBegin(GL_POINTS);
	for (int i = 0; i < colorPointCount; i++)
	{
		glColor3ub(pBufferColor[4 * i], pBufferColor[4 * i + 1], pBufferColor[4 * i + 2]);
		GLfloat pX = pCSPoints[i].X;
		GLfloat pY = pCSPoints[i].Y;
		GLfloat pZ = -pCSPoints[i].Z;
		glVertex3f(pX, pY, pZ);
	}
	glEnd();
	glPopMatrix();
}

void Draw3DLine()
{

}

void Draw3DPlane()
{

}

void DrawMeshes()
{
	/**************/
	/*************/
	if (!BG_IS_OPEN)
	{
		glPushMatrix();
		glBegin(GL_TRIANGLE_STRIP/*GL_TRIANGLES*/);
		for (int i = 0; i < colorPointCount; i += 3)//3计
		{

			glColor3ub(pBufferColor[4 * i], pBufferColor[4 * i + 1], pBufferColor[4 * i + 2]);
			if (/*i % iWidthColor < (iWidthColor - 1) &&*/ i < iWidthColor*(iHeightColor - 1))
			{
				glVertex3f(pCSPoints[i].X, pCSPoints[i].Y, -pCSPoints[i].Z);
				glVertex3f(pCSPoints[i + iWidthColor].X, pCSPoints[i + iWidthColor].Y, -pCSPoints[i + iWidthColor].Z);
			}
			else
			{
				i += iWidthColor;
			}
		}
		glEnd();
		//glDisable(GL_BLEND);
		glPopMatrix();
	}
	else
	{
		glPushMatrix();
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_TRIANGLE_STRIP);
		for (int i = 0; i < colorPointCount; i += 3)
		{
			glColor4ub(pBufferColor[4 * i], pBufferColor[4 * i + 1], pBufferColor[4 * i + 2], 125);
			if (i % iWidthColor < (iWidthColor - 1) && i < iWidthColor*(iHeightColor - 1))
			{
				glVertex3f(pCSPoints[i].X, pCSPoints[i].Y, -pCSPoints[i].Z);
				glVertex3f(pCSPoints[i + iWidthColor].X, pCSPoints[i + iWidthColor].Y, -pCSPoints[i + iWidthColor].Z);

				glVertex3f(pCSPoints[i + 1].X, pCSPoints[i + 1].Y, -pCSPoints[i + 1].Z);
				glVertex3f(pCSPoints[i + iWidthColor + 1].X, pCSPoints[i + iWidthColor + 1].Y, -pCSPoints[i + iWidthColor + 1].Z);
			}
			else
			{
				i += iWidthColor;
			}
		}
		glEnd();
		glPopMatrix();
	}
}

int glPrintf(const char *format, ...)
{
	char buffer[65536];
	va_list args;                         //define argument list
	va_start(args, format);               //get this function's argument list
	vsprintf_s(buffer, format, args);     //give argument list to vsprintf(), and save string into buffer
	va_end(args);
	static int x0 = 0, y0 = 0;
	if (base == 0)                        //if we don't have base, execute this part
	{
		base = glGenLists(65536);
		hFont = CreateFont(nHeight, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, fdwCharSet,
			OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
			FF_DONTCARE | DEFAULT_PITCH, szFace);
	}
	float p0[4], p1[4], c0[2][4] = { { 0, 0, 0, 1 }, { 1, 1, 1, 1 } };
	int i, j, len, offset[2] = { 1, 0 };
	wchar_t *wstr;
	GLint viewport[4];
	HDC hdc = 0;
	glGetIntegerv(GL_VIEWPORT, viewport);                  // get view-port, set projection matrix needed
	glGetFloatv(GL_CURRENT_RASTER_POSITION, p0);           // get raster position, use this to calculate the coordinate when changing line
	glPushAttrib(GL_LIGHTING_BIT | GL_DEPTH_BUFFER_BIT);   // push attributes, be prepared for initialization
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);                           // setting: projection matrix
	glPushMatrix();                                        // be prepared for projection matrix 
	glLoadIdentity();
	gluOrtho2D(0, viewport[2], 0, viewport[3]);
	glMatrixMode(GL_MODELVIEW);                             // custom setting model-view matrix
	glPushMatrix();                                         // be prepared for model-view matrix 
	glLoadIdentity();
	if (memcmp(buffer, ">>glut", 6) == 0)                      // if the input string is ">>glut", set the start point of article in the left-top point.
	{
		sscanf_s(buffer, ">>glut(%i,%i)", &x0, &y0);
		glTranslatef(x0, -y0, 0);
		glRasterPos2f(4, viewport[3] - nHeight);
		//glRasterPos2f(50.0f, 20.0f);
		//cout << endl << "4 " << viewport[3] - nHeight;
	}
	else if (strcmp(buffer, ">>free") == 0)                 // if the input string is ">>free", delete the string (The sample did not use others）
	{
		glDeleteLists(base, 65536); base = 0;
		memset(loaded, 0, 65536 * sizeof(bool));
		DeleteObject(hFont);
	}
	else
	{
		glRasterPos2f(p0[0], p0[1]);                                             // (这句是没实质作用的, 不过, 要先呼叫它, 下一句 glGetFloatv() 才能正确执行 )
		glGetFloatv(GL_CURRENT_RASTER_COLOR, c0[1]);                             // 最得 glColor() 所述的颜色
		len = MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, buffer, -1, 0, 0);     // 计算转换成大五码后, 字符串的长度
		wstr = (wchar_t*)malloc(len*sizeof(wchar_t));                            // 配置内存去储存 大五码字符串
		MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, buffer, -1, wstr, len);      // 把 buffer 转换成 大五码字符串
		for (j = 0; j < 2; j++)                                                  // 秀两次字符串
		{
			glColor4fv(c0[j]);                                     //设定 字符串颜色
			//glColor3b(255, 0, 0);
			glRasterPos2f(p0[0] + offset[j], p0[1] - offset[j]);   //设定 字符串起始位置
			for (i = 0; i < len - 1; i++)
			{
				if (wstr[i] == '\n')                                        // 如果是 '\n', 就换新行
				{
					glGetFloatv(GL_CURRENT_RASTER_POSITION, (float*)&p1);   //取得现在的位置
					//glRasterPos2f(4 + offset[j], p1[1] - (nHeight + 2));    //设定新行的起始位置
					glRasterPos2f(4 + x0 + offset[j], p1[1] - (nHeight + 2));
				}
				else
				{
					if (!loaded[wstr[i]])    // 如果字符未被加载
					{
						if (hdc == 0)
						{
							hdc = wglGetCurrentDC();    // 取得 device context 的 handle (窗口系统的东西)
							SelectObject(hdc, hFont);   // 选取 font handle (窗口系统的东西)
						}
						wglUseFontBitmapsW(hdc, wstr[i], 1, base + wstr[i]);    // 加载字符的字型图像
						loaded[wstr[i]] = TRUE;                                 // 标示这字符已被加载
					}
					glCallList(base + wstr[i]);   //  绘画字符的 display list
				}
			}
		}
		free(wstr);    // 把配置了的内存归还系统
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
	return 0;
}
#pragma endregion OpenGL Function

#pragma region Mt232 Function
void MtReflashWait()
{
	int i = mdr.wdt;
	int timeout = 0;
	while (i == mdr.wdt && timeout < 50)
	{
		MtReflash(md);
		timeout += 1;
		Sleep(1);
	}
}

/*add one MtHome button to the keyboard function*/
bool MtHome()
{
	MtCmd("mt_emg 0");
	Sleep(100);
	MtCmd("mt_delay 20");
	MtCmd("mt_m_acc 20");
	MtCmd("mt_v_acc 20");
	MtCmd("mt_speed 30");
	MtCmd("mt_check_ot 0,0,0,0");
	MtCmd("mt_set_home_acc 50");
	MtCmd("mt_leave_home_speed 2,2,2,2");
	MtCmd("mt_go_home 50,50,10,50,2,2,1,255"); //speed to back home(speed x, speed y, speed z, speed u, order x, order y, order z, order u, 255 = u do not back home 
	Sleep(1000);
	do{
		MtReflashWait();
		if (MtFlag(MotionStatus::emg_signal))
		{
			MtCmd("mt_abort_home");
			return false;
		}
	} while (MtFlag(MotionStatus::home_x) || MtFlag(MotionStatus::home_y) || MtFlag(MotionStatus::home_z)); //home_x 回到原点的过程 = true
	MtCmd("mt_home_finish");
	MtCmd("mt_speed1 30");
	MtCmd("mt_soft_limit 0,-2,300"); //Axe no., Left limit, Right limit
	MtCmd("mt_soft_limit 1,-2,300");
	MtCmd("mt_soft_limit 2,-2,100");
	MtCmd("mt_soft_limit 3,-450,450");
	MtCmd("mt_check_ot 1,1,1,0"); //enable limit (by Axi no)
	MtCmd("mt_out 11,1"); //door switch
	MtCmd("mt_m_acc 150");  //door switch
	MtCmd("mt_v_acc 80");  //door switch
	return true;
}

void MtInit(void)
{
	array<System::String^>^ serialPorts = nullptr;
	try
	{
		// Get a list of serial port names.
		serialPorts = SerialPort::GetPortNames();
	}
	catch (Win32Exception^ ex)
	{
		Console::WriteLine(ex->Message);
	}
	Console::WriteLine("The following serial ports were found:");
	// Display each port name to the console.
	for each(System::String^ port in serialPorts)
	{
		Console::WriteLine(port);
	}
	//cout << MtTestDebby(2, 3) << endl;
	char ptr[5] = "COM6";
	//cout << "Connect I/O = " << MtConnect(ptr) << endl;
	long IS_CONECT = 0;
	//
	do
	{
		IS_CONECT = MtConnect(ptr);
		cout << "Connect I/O = " << IS_CONECT << endl;
	} while (IS_CONECT != 1);
	Sleep(50);
	long MtEmpty();
	bool IS_HOME = MtHome();
}

void MtMove(void)
{
	//bool IS_HOME = MtHome();
	MtReflash(md);
	cout << "md->x : " << md->x << endl;
	cout << "md->y : " << md->y << endl;
	Sleep(100);
	for (int i = 0; i < 5; i++)
	{
		MtCmd("mt_out 12,1");
		Sleep(100);
		MtCmd("mt_out 12,0");
		Sleep(100);
	}
	for (int i = 0; i < ROIStorageCount; i++)
	{
		char mybuffx[50], mybuffy[50], mybuffz[50];
		char commandx[60] = "mt_m_x ", commandy[60] = "mt_m_y ", commandz[60] = "mt_m_z ";
		switch (STORAGE_TYPE)
		{
		case 0:
			sprintf(mybuffx, "%f", ROICameraSP_MachineCoord_Storage[i].X * 1000 - 172);
			sprintf(mybuffy, "%f", ROICameraSP_MachineCoord_Storage[i].Y * 1000 - 158);
			sprintf(mybuffz, "%f", ROICameraSP_MachineCoord_Storage[i].Z * 1000 + 400/*- ROICameraSP_MachineCoord_Storage[i].Y * 1000 * tan(dev_theta)*/);
			break;
		case 1:
			sprintf(mybuffx, "%f", ROICameraSP_MachineCoord_Proj_Storage[i].X * 1000 - 172);
			sprintf(mybuffy, "%f", ROICameraSP_MachineCoord_Proj_Storage[i].Y * 1000 - 158);
			sprintf(mybuffz, "%f", ROICameraSP_MachineCoord_Proj_Storage[i].Z * 1000 + 400 /*- ROICameraSP_MachineCoord_Proj_Storage[i].Y * 1000 * tan(dev_theta)*/);
			break;
		}
		strcat(commandx, mybuffx);
		strcat(commandy, mybuffy);
		strcat(commandz, mybuffz);

		MtCmd(commandx);
		MtCmd(commandy);
		MtCmd(commandz);

	}
	Thread^ thread1 = gcnew Thread(gcnew ThreadStart(MtCheck));
	thread1->Name = "thread1";
	thread1->Start();
	MtCmd("mt_m_z 0");
}

void MtCheck(void)
{
	for (mtmove_step = 0; mtmove_step < ROIStorageCount; mtmove_step++)
	{
		float value = 0;
		switch (STORAGE_TYPE)
		{
		case 0:
			value = ROICameraSP_MachineCoord_Storage[mtmove_step].Z * 1000 - ROICameraSP_MachineCoord_Storage[mtmove_step].Y * 1000 * tan(dev_theta);
			break;
		case 1:
			value = ROICameraSP_MachineCoord_Proj_Storage[mtmove_step].Z * 1000 - ROICameraSP_MachineCoord_Proj_Storage[mtmove_step].Y * 1000 * tan(dev_theta);
			break;
		}
		do
		{
			MtReflash(md);
			//cout << value << " " << md->z << endl;
		} while (abs(md->z - value) > 0.01/*md->z!=value*/);
		//cout << value << " " << md->z << endl;
		//Sleep(2000);
	}
	mtmove_step = ROIStorageCount - 1;
}
#pragma endregion Mt232 Function

#pragma region PortAudio Function
/*------------------------------------------------------------*/
/*-----------------PortAudio Function Declare-----------------*/
/*------------------------------------------------------------*/
bool InitPortAudio()
{
	if (paInit.result() != paNoError) goto error;
	if (sine.open(Pa_GetDefaultOutputDevice()))
	{
		IS_PORTAUDIO_INIT = TRUE;
		return IS_PORTAUDIO_INIT;
	}
error:
	fprintf(stderr, "An error occurred while using the PortAudio stream\n");
	fprintf(stderr, "Error number: %d\n", paInit.result());
	fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(paInit.result()));
	return 0;
}
#pragma endregion PortAudio Function

#pragma region Path Generation Function
/*--------------------------------------------------------*/
/*----------------Path Generation Function----------------*/
/*--------------------------------------------------------*/
void PathGenProjToPlane(CameraSpacePoint* Data, int DataNum, double* PlaneCoeff, CameraSpacePoint* Result)
{
	for (int i = 0; i < DataNum; i++)
	{
		double tScal = -(PlaneCoeff[0] * Data[i].X + PlaneCoeff[1] * Data[i].Y + PlaneCoeff[2] * Data[i].Z + PlaneCoeff[3]) / (pow(PlaneCoeff[0], 2) + pow(PlaneCoeff[1], 2) + pow(PlaneCoeff[2], 2));
		Result[i].X = Data[i].X + tScal * PlaneCoeff[0];
		Result[i].Y = Data[i].Y + tScal * PlaneCoeff[1];
		Result[i].Z = Data[i].Z + tScal * PlaneCoeff[2];
		//cout << endl << Result[i].X << " " << Result[i].Y << " " << Result[i].Z << endl;
	}
}
#pragma endregion Path Generation Function

#pragma region AR Function
/*---------------------------------------------------*/
/*----------------AR Function Declare----------------*/
/*---------------------------------------------------*/
float Dot(float* Vector1, float* Vector2)
{
	return Vector1[0] * Vector2[0] + Vector1[1] * Vector2[1] + Vector1[2] * Vector2[2];
}

float Determinant(float a, float b, float c, float d)
{
	return a*d - b*c;
}
#pragma endregion AR Function

#pragma endregion Function