#pragma region Region_Library
/*--------------Standard Library-------------*/
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
#pragma comment(lib,"opengl32.lib")  
#pragma comment(lib,"GLU32.LIB")  
#pragma comment(lib,"GLUT32.LIB")
/*--------------ARTool Kit-------------*/
//#include <AR/gsub.h>
//#include <AR/video.h>
//#include <AR/param.h>
//#include <AR/ar.h>
/*--------------OpenHaptics-------------*/
//#include <HL/hl.h>
//#include <HDU/hduMatrix.h>
//#include <HDU/hduError.h>
//#include <HLU/hlu.h>
/*--------------LoadObj-------------*/
#include "GLM.h"
/*--------------Beep Sound-------------*/
//#pragma comment(lib,"winmm.lib") 
//#include <mmsystem.h> 
#include "portaudio.h"
using namespace std;
using namespace cv;
#include "Declae.h"
/*--------------M232-------------*/
//#include "dllmain.h"
#using <System.dll>
using namespace System;
using namespace System::IO::Ports;
using namespace System::ComponentModel;
#pragma endregion Region_Library
/*------------------------------------------*/
/*--------------OpenGL Function-------------*/
/*------------------------------------------*/
/*Check it already! 1st*/
void DrawCubic()
{   /*Virtual object's translate and rotate function*/
	glPushMatrix();
	glTranslatef(CubicPosi.x, CubicPosi.y, CubicPosi.z);
	glTranslatef(ObjPosi.x, ObjPosi.y, ObjPosi.z);
	/*Intersect: the center red point of three-Axis dispenser's translate value*/
	glTranslatef(Intersect.X, Intersect.Y, Intersect.Z);
	glMultMatrixf(M_Cubic);
	//glRotatef(-90, 0, 0, 1);
	/*glEnable()用来启用各种功能，功能由参数决定*/
	glEnable(GL_TEXTURE_2D);
	/*glEnableClientState开启顶点数组功能*/
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	/*If the collide not happened, we draw the cubic with the original color*/
	if (!ROI_IS_COLLIDE)
	{
		GLMgroup* group = OBJ->groups;
		int gCount = 0;
		while (group)
		{
			//if (gCount != 4)
			//{
			//	gCount++;
			//	group = group->next;
			//	continue;
			//}
			glColor3f(1, 1, 1);
			glVertexPointer(3, GL_FLOAT, 0, vertices[gCount]);
			/*glTexCoordPointer(int size, int type, int stride, Buffer Point);设置顶点数组为纹理坐标缓存
			http://blog.csdn.net/lydia5945/article/details/7254367 */
			glTexCoordPointer(2, GL_FLOAT, 0, vtextures[gCount]);
			glBindTexture(GL_TEXTURE_2D, textures[6 - gCount]);
			glDrawArrays(GL_TRIANGLES, 0, group->numtriangles * 3);
			gCount++;
			group = group->next;
		}
	}
	else
	{
		GLMgroup* group = OBJ->groups;
		int gCount = 0;
		while (group)
		{
			/*---------------------------------------------------------------------*/
			/*----------------Now, our method is to draw the color cubic and draw the
			WireFrame of the cubic, need to have a better method-------------------*/
			/*---------------------------------------------------------------------*/
			glVertexPointer(3, GL_FLOAT, 0, vertices[gCount]);
			glColor3f(1, 0, 0);
			glDrawArrays(GL_TRIANGLES, 0, group->numtriangles * 3);
			glColor3f(0, 0, 0);
			glLineWidth(2.5);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawArrays(GL_TRIANGLES, 0, group->numtriangles * 3);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			gCount++;
			group = group->next;
		}
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

/*Rotate and translate in special key can move virtual objects by hand */
void SpecialKeys(int key, int x, int y)
{
	//Press F1 to store pt
	if (key == GLUT_KEY_F1 && !ROI_IS_COLLIDE && ARFunc_InsideTriCheck)
	{
		ROICameraSPStorage();
	}
	//Press F4 to enable or disable transparent mode
	else if (key == GLUT_KEY_F4)
	{
		if (Cubic_IS_BLEND)
		{
			Cubic_IS_BLEND = FALSE;
			cout << endl << "Disable Transparent Mode..." << endl;
		}
		else
		{
			Cubic_IS_BLEND = TRUE;
			cout << endl << "Enable Transparent Mode..." << endl;
		}
	}
	else if (key == GLUT_KEY_F5)
	{
		bool IS_HOME = MtHome();
		cout << endl << "Let the machine come back home now..." << endl;
	}
	else if (key == GLUT_KEY_F2)/*Use F2 to delete the storaged points*/
	{
		if (ROICameraSP_Storage != nullptr)
		{
			delete[] ROICameraSP_Storage;
		}
		if (ROICameraSP_Proj_Storage != nullptr)
		{
			delete[] ROICameraSP_Proj_Storage;
		}
		if (ROICameraSP_MechCoord_Storage != nullptr)
		{
			delete[] ROICameraSP_MechCoord_Storage;
		}
		if (ROICameraSP_MechCoord_Proj_Storage != nullptr)
		{
			delete[] ROICameraSP_MechCoord_Proj_Storage;
		}
		ROIStorageCount = 0;
	}
	/*-------------Rotation------------*/
	//else if (key == GLUT_KEY_DOWN && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.x == 360)
	//	{
	//		CubicRota.x = 0;
	//	}
	//	else
	//	{
	//		CubicRota.x += 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.x : " << CubicRota.x << endl;
	//}
	//else if (key == GLUT_KEY_UP && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.x == -360)
	//	{
	//		CubicRota.x = 0;
	//	}
	//	else
	//	{
	//		CubicRota.x -= 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.x : " << CubicRota.x << endl;
	//}
	//else if (key == GLUT_KEY_RIGHT && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.y == 360)
	//	{
	//		CubicRota.y = 0;
	//	}
	//	else
	//	{
	//		CubicRota.y += 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.y : " << CubicRota.y << endl;
	//}
	//else if (key == GLUT_KEY_LEFT && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.y == -360)
	//	{
	//		CubicRota.y = 0;
	//	}
	//	else
	//	{
	//		CubicRota.y -= 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.y : " << CubicRota.y << endl;
	//}
	//else if (key == GLUT_KEY_F9 && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.z == 360)
	//	{
	//		CubicRota.z = 0;
	//	}
	//	else
	//	{
	//		CubicRota.z += 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.z : " << CubicRota.z << endl;
	//}
	//else if (key == GLUT_KEY_F10 && !IS_PathGenProcessing)
	//{
	//	if (CubicRota.z == -360)
	//	{
	//		CubicRota.z = 0;
	//	}
	//	else
	//	{
	//		CubicRota.z -= 1.0f;
	//	}
	//
	//	cout << endl << "CubicRota.z : " << CubicRota.z << endl;
	//}
	/*-------------Translation------------*/
	else if (key == GLUT_KEY_DOWN)
	{
		CubicPosi.y += 0.001f;
		cout << endl << "CubicPosi.y : " << CubicPosi.y << endl;
	}
	else if (key == GLUT_KEY_UP)
	{
		CubicPosi.y -= 0.001f;
		cout << endl << "CubicPosi.y : " << CubicPosi.y << endl;
	}
	else if (key == GLUT_KEY_RIGHT)
	{
		CubicPosi.x += 0.001f;
		cout << endl << "CubicPosi.x : " << CubicPosi.x << endl;
	}
	else if (key == GLUT_KEY_LEFT)
	{
		CubicPosi.x -= 0.001f;
		cout << endl << "CubicPosi.x : " << CubicPosi.x << endl;
	}
	else if (key == GLUT_KEY_F9)
	{
		CubicPosi.z += 0.001f;
		cout << endl << "CubicPosi.z : " << CubicPosi.z << endl;
	}
	else if (key == GLUT_KEY_F10)
	{
		CubicPosi.z -= 0.001f;
		cout << endl << "CubicPosi.z : " << CubicPosi.z << endl;
	}
	/*--------------Re write the screen display--------------*/
	glutPostRedisplay();
}

/*Check it already! 2nd*/
void RenderScene()
{
	/*--------------RenderScene Background & Drawing Type-------------*/
	if (BG_IS_OPEN)
	{
		SceneWithBackground();
		switch (DRAWING_TYPE) {
		case 0:
			break;
		case 1:
			//cout << endl << "Error in drawing type" << endl;
			DrawMeshes();
			break;
		case 2:
			DrawPointCloud();
			break;
		}
	}
	else
	{
		SceneWithoutBackground();
		switch (DRAWING_TYPE) {
		case 0:
			cout << endl << "Error in drawing type" << endl;
			break;
		case 1:
			DrawMeshes();
			break;
		case 2:
			DrawPointCloud();
			break;
		}
	}
	/*--------------Draw Probe Tip-------------*/
	if (ROIDepthCount != 0)
	{
		glPushMatrix();
		glPointSize(2.0f);
		glBegin(GL_POINTS);
		for (int i = 0; i < ROIDepthCount; i++)
		{
			glColor3ub(0, 255, 0);
			/*draw probe tip's depth points*/
			glVertex3f(ROICameraSP[i].X, ROICameraSP[i].Y, ROICameraSP[i].Z);
		}
		glEnd();
		glPopMatrix();
	}
	/**/
	/*--------------Collision Detection-------------*/
	if (ROIDepthCount != 0 && ARFunc_IS_ON)
	{
		GLfloat TransM[16] = { 0 };
		TransM[0] = TransM[5] = TransM[10] = TransM[15] = 1;
		glPushMatrix();
		glLoadIdentity();
		//glRotatef(90, 0, 0, 1);
		glMultMatrixf(M_Cubic_inv);
		glTranslatef(-Intersect.X, -Intersect.Y, -Intersect.Z);
		glTranslatef(-ObjPosi.x, -ObjPosi.y, -ObjPosi.z);
		glGetFloatv(GL_MODELVIEW_MATRIX, TransM);
		glPopMatrix();
		if (ROICameraSP_TouchDetec != nullptr)
		{
			delete[] ROICameraSP_TouchDetec;
		}
		ROICameraSP_TouchDetec = new CameraSpacePoint[ROIDepthCount];
		/*Move the probe tip to the */
		ROITrans(ROICameraSP, ROIDepthCount, TransM, ROICameraSP_TouchDetec);
		int CollideCount = 0;
		int CollideCount_Circle = 0;
		for (int ROICount = 0; ROICount < ROIDepthCount; ROICount++)
		{
			GLMgroup* group = OBJ->groups;
			int gCount = 0;
			while (group)
			{
				for (int TriCount = 0; TriCount < group->numtriangles; TriCount++)
				{
					float vx = ROICameraSP_TouchDetec[ROICount].X - vertices[gCount][TriCount * 9 + 0];
					float vy = ROICameraSP_TouchDetec[ROICount].Y - vertices[gCount][TriCount * 9 + 1];
					float vz = ROICameraSP_TouchDetec[ROICount].Z - vertices[gCount][TriCount * 9 + 2];

					float nx = normals[gCount][TriCount * 9 + 0];
					float ny = normals[gCount][TriCount * 9 + 1];
					float nz = normals[gCount][TriCount * 9 + 2];

					float Normalized_V = pow(pow(vx, 2) + pow(vy, 2) + pow(vz, 2), -0.5);
					float Normalized_N = pow(pow(nx, 2) + pow(ny, 2) + pow(nz, 2), -0.5);
					float dot = vx*nx*Normalized_V*Normalized_N + vy*ny*Normalized_V*Normalized_N + vz*nz*Normalized_V*Normalized_N;
					float Psi = acos(dot);
					//cout << Psi*180.0f / 3.14159265358979323846 << endl;
					if (Psi*180.0f / 3.14159265358979323846 > 90.0f)
					{
						switch (gCount)
						{
						case 3:
							/*the circle of the obj*/
							CollideCount_Circle++;
							break;
						default:
							CollideCount++;
							break;
						}
						//glPushMatrix();
						//glColor3ub(255, 0, 0);
						//glTranslatef(0.01, -0.451, 0.2);
						//glTranslatef(Intersect.X, Intersect.Y, Intersect.Z);
						//glMultMatrixf(M_Cubic);
						//glRotatef(-90, 0, 0, 1);
						//glBegin(/*GL_TRIANGLES*/GL_LINE_LOOP);
						//glVertex3f(vertices[gCount][TriCount * 9 + 0], vertices[gCount][TriCount * 9 + 1], vertices[gCount][TriCount * 9 + 2]);
						//glVertex3f(vertices[gCount][TriCount * 9 + 3], vertices[gCount][TriCount * 9 + 4], vertices[gCount][TriCount * 9 + 5]);
						//glVertex3f(vertices[gCount][TriCount * 9 + 6], vertices[gCount][TriCount * 9 + 7], vertices[gCount][TriCount * 9 + 8]);
						//glEnd();
						//glPopMatrix();
						//cout << TriCount << " " << CollideCount << endl;
					}
				}
				gCount++;
				group = group->next;
			}
			if (CollideCount_Circle == 0)
			{
				ROI_IS_COLLIDE = FALSE;
			}
			else if (CollideCount >= OBJ->numtriangles - 76)
			{
				//cout << "Collide" << endl;
				ROI_IS_COLLIDE = TRUE;
				break;
			}
			CollideCount = 0;
			CollideCount_Circle = 0;
		}
	}
	/*--------------AR Function-------------*/
	/*find depth points and do not collide, call ARFunc_FindProj()*/
	if (ROIDepthCount > 0 && !ROI_IS_COLLIDE)
	{
		/*use this to get the projection point*/
		ARFunc_FindProj();
		if (IS_ARFunc_InsideTriCheck && ARFunc_IS_ON)
		{
			glPushMatrix();
			glPointSize(5.0f);
			glColor3ub(255, 0, 0);
			glBegin(GL_POINTS);
			glVertex3f(ARFunc_ROICSP_Proj->X, ARFunc_ROICSP_Proj->Y, ARFunc_ROICSP_Proj->Z);
			glEnd();
			glPopMatrix();
			/*calculate the distance between projection point and center point*/
			//unit in m
			ARFunc_ROICSP_Proj_Dist = pow(pow(ARFunc_ROICSP_Proj->X - ROICenterCameraS.x, 2) + pow(ARFunc_ROICSP_Proj->Y - ROICenterCameraS.y, 2) + pow(ARFunc_ROICSP_Proj->Z - ROICenterCameraS.z, 2), 0.5);
			float dist = ARFunc_ROICSP_Proj_Dist * 1000;
			const int low_range = 30, high_range = 100;
			int low_r = 1, high_r = 0, low_g = 0, high_g = 1;
			float rr = 0, gg = 0, bb = 0;
			/*change the color of the line by the distance range*/
			if (dist > high_range)
			{
				rr = 0;
				gg = 1;
				bb = 0;
			}
			else if (dist < low_range)
			{
				rr = 1;
				gg = 0;
				bb = 0;
			}
			else
			{
				rr = (dist - low_range) / ((low_range + high_range) - low_range) * (high_r - low_r) + low_r;
				gg = (dist - (low_range + high_range)) / (high_range - (low_range + high_range)) * (high_g - low_g) + low_g;
				bb = -1.0f / pow((high_range - low_range) / 2, 2) * pow((dist - (low_range + high_range) / 2), 2) + 1.0f;
			}
			//cout << rr << " " << gg << " " << bb << endl;
			/* draw line */
			glEnable(GL_LINE_STIPPLE);
			glPushMatrix();
			glColor3f(rr, gg, bb);
			glLineWidth(2.5);
			/*glLineStipple划线的模式，虚线*/
			glLineStipple(1, 0x1C47);
			glBegin(GL_LINES);
			glVertex3f(ARFunc_ROICSP_Proj->X, ARFunc_ROICSP_Proj->Y, ARFunc_ROICSP_Proj->Z);
			glVertex3f(ROICenterCameraS.x, ROICenterCameraS.y, ROICenterCameraS.z);
			glEnd();
			glPopMatrix();
			glDisable(GL_LINE_STIPPLE);
			if (IS_PORTAUDIO_INIT)
			{
				IS_PORTAUDIO_START = sine.start();
			}
		}
		else
		{
			if (IS_PORTAUDIO_START == 0)
			{
				/* sine 声音*/
				sine.stop();
			}
		}
	}
	else
	{
		if (IS_PORTAUDIO_START == 0)
		{
			sine.stop();
		}
	}
	/*--------------Transfer Prob Tip from GL Coordinate to Machine Coordinate-------------*/
	if (ROIDepthCount > 0 && ROICameraSP != nullptr && !ROI_IS_COLLIDE)
	{
		GLfloat TransM_toMechCoord[16] = { 0 };
		TransM_toMechCoord[0] = TransM_toMechCoord[5] = TransM_toMechCoord[10] = TransM_toMechCoord[15] = 1;
		glPushMatrix();
		glLoadIdentity();
		glTranslatef(TipPosi.x, TipPosi.y, TipPosi.z);
		glRotatef(-90, 1, 0, 0);
		//glRotatef(90, 0, 0, 1);
		glMultMatrixf(M_Cubic_inv);
		glTranslatef(-Intersect.X, -Intersect.Y, -Intersect.Z);
		glGetFloatv(GL_MODELVIEW_MATRIX, TransM_toMechCoord);
		glPopMatrix();
		if (ROICameraSP_MechCoord != nullptr)
		{
			delete[] ROICameraSP_MechCoord;
		}
		if (ROICameraSP_Proj_MechCoord != nullptr)
		{
			delete[] ROICameraSP_Proj_MechCoord;
		}
		ROICameraSP_MechCoord = new CameraSpacePoint[1];
		ROICameraSP_Proj_MechCoord = new CameraSpacePoint[1];
		/*normal GL coordinate*/
		ROICameraSP_MechCoord->X = ROICenterCameraS.x;
		ROICameraSP_MechCoord->Y = ROICenterCameraS.y;
		ROICameraSP_MechCoord->Z = ROICenterCameraS.z;
		/*After projection coordinate*/
		ROICameraSP_Proj_MechCoord->X = ARFunc_ROICSP_Proj->X;
		ROICameraSP_Proj_MechCoord->Y = ARFunc_ROICSP_Proj->Y;
		ROICameraSP_Proj_MechCoord->Z = ARFunc_ROICSP_Proj->Z;
		ROITrans(ROICameraSP_MechCoord, 1, TransM_toMechCoord, ROICameraSP_MechCoord);
		ROITrans(ROICameraSP_Proj_MechCoord, 1, TransM_toMechCoord, ROICameraSP_Proj_MechCoord);
		ROICameraSP_MechCoord->Y = -ROICameraSP_MechCoord->Y;
		ROICameraSP_Proj_MechCoord->Y = -ROICameraSP_Proj_MechCoord->Y;
	}
	/* F1 button calls this */
	/*--------------Draw Storage Point-------------*/
	if (ROIStorageCount > 0)
	{
		//cout << ROIStorageCount << endl;
		glPushMatrix();
		glPointSize(5.0f);
		if (CUBIC_MOVE)
		{
			glTranslatef(DeviaDueToY->X, DeviaDueToY->Y, DeviaDueToY->Z);
		}
		glColor3ub(255, 0, 137);
		glBegin(GL_POINTS);
		GLfloat pX = 0, pY = 0, pZ = 0;
		switch (STORAGE_TYPE)
		{
		case 0:
			for (int i = 0; i < ROIStorageCount; i++)
			{
				pX = ROICameraSP_Storage[i].X;
				pY = ROICameraSP_Storage[i].Y;
				pZ = ROICameraSP_Storage[i].Z;
				glVertex3f(pX, pY, pZ);
			}
			break;
		case 1:
			for (int i = 0; i < ROIStorageCount; i++)
			{
				pX = ROICameraSP_Proj_Storage[i].X;
				pY = ROICameraSP_Proj_Storage[i].Y;
				pZ = ROICameraSP_Proj_Storage[i].Z;
				glVertex3f(pX, pY, pZ);
			}
			break;
		}
		glEnd();
		glPopMatrix();
	}
	/*--------------cubic movement deviation--------------*/
	/*--------------Calculate deviation & Draw Cubic-------------*/
	CameraSpacePoint* Y_Devia = new CameraSpacePoint[2];
	Y_Devia[0].X = Y_Devia[0].Y = Y_Devia[0].Z = 0;
	Y_Devia[1].X = Y_Devia[1].Y = Y_Devia[1].Z = 0;
	//MtReflash(md);
	if (ROIStorageCount > 0 && CUBIC_MOVE)
	{
		float dy = 0;
		switch (STORAGE_TYPE)
		{
		case 0:
			dy = ROICameraSP_MechCoord_Storage[mtmove_step].Y;
			break;
		case 1:
			dy = ROICameraSP_MechCoord_Proj_Storage[mtmove_step].Y;
			break;
		}
		GLfloat TransM_OBJ[16] = { 0 };
		TransM_OBJ[0] = TransM_OBJ[5] = TransM_OBJ[10] = TransM_OBJ[15] = 1;
		glPushMatrix();
		glLoadIdentity();
		glTranslatef(Intersect.X, Intersect.Y, Intersect.Z);
		glMultMatrixf(M_Cubic);
		//glRotatef(-90, 0, 0, 1);
		glRotatef(90, 1, 0, 0);
		glTranslatef(TipPosi.x, TipPosi.y, TipPosi.z);
		glGetFloatv(GL_MODELVIEW_MATRIX, TransM_OBJ);
		glPopMatrix();
		Y_Devia[1].Y = dy;
		ROITrans(Y_Devia, 2, TransM_OBJ, Y_Devia);
		DeviaDueToY->X = Y_Devia[1].X - Y_Devia[0].X;
		DeviaDueToY->Y = Y_Devia[1].Y - Y_Devia[0].Y + (Y_Devia[1].Z - Y_Devia[0].Z)*tan(-dev_theta);
		DeviaDueToY->Z = Y_Devia[1].Z - Y_Devia[0].Z;
		//cout << Y_Devia[0].X << " " << Y_Devia[0].Y << " " << Y_Devia[0].Z << endl;
		//cout << Y_Devia[1].X << " " << Y_Devia[1].Y << " " << Y_Devia[1].Z << endl;
	}
	glPushMatrix();
	if (Cubic_IS_BLEND)
	{
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	glTranslatef(DeviaDueToY->X, DeviaDueToY->Y, DeviaDueToY->Z);
	/*draw the cubic in here:DCH*/
	DrawCubic();
	ARFunc_IS_ON = FALSE;
	if (Cubic_IS_BLEND)
	{
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
	}
	glPopMatrix();
	//delete[] Y_Devia;
	/*--------------Show Prob Tip Coordinate on Screen-------------*/
	glPushMatrix();
	glColor3ub(0, 255, 0);
	/*glPrintF is used to draw the probe tip's coordinate position*/
	if (ROICameraSP_MechCoord != nullptr)
	{
		//glPrintf(">>glut(%i, %i)", -ROICenterColorS_Old.x - 60, ROICenterColorS_Old.y - 35);
		glPrintf(">>glut(%i, %i)", ROICenterColorS_Old.x - 60, ROICenterColorS_Old.y - 35);
		glPrintf("(%.3f, %.3f, %.3f)", ROICameraSP_MechCoord[0].X, ROICameraSP_MechCoord[0].Y, ROICameraSP_MechCoord[0].Z);
	}
	glPopMatrix();
	if (ARFunc_IS_ON)
	{
		/*--------------Change Color of projection line-------------*/
		float dist = ARFunc_ROICSP_Proj_Dist * 1000;
		const int low_range = 30, high_range = 100;
		int low_r = 1, high_r = 0, low_g = 0, high_g = 1;
		float rr = 0, gg = 0, bb = 0;
		if (dist > high_range)
		{
			rr = 0;
			gg = 1;
			bb = 0;
		}
		else if (dist < low_range)
		{
			rr = 1;
			gg = 0;
			bb = 0;
		}
		else
		{
			rr = (dist - low_range) / ((low_range + high_range) - low_range) * (high_r - low_r) + low_r;
			gg = (dist - (low_range + high_range)) / (high_range - (low_range + high_range)) * (high_g - low_g) + low_g;
			bb = -1.0f / pow((high_range - low_range) / 2, 2) * pow((dist - (low_range + high_range) / 2), 2) + 1.0f;
		}
		/*--------------Show Prob Tip Coordinate on Screen-------------*/
		glPushMatrix();
		glColor3f(rr, gg, bb);
		if (IS_ARFunc_InsideTriCheck && !ROI_IS_COLLIDE)
		{
			glPrintf(">>glut(%i, %i)", ROICenterColorS_Old.x + 10, ROICenterColorS_Old.y + 15);
			glPrintf("%.2f mm ", ARFunc_ROICSP_Proj_Dist * 1000);
		}
		glPopMatrix();
	}
	ROI_IS_COLLIDE = FALSE;
	if (Finish_Without_Update)
		glFinish();
	else
		glutSwapBuffers();
	FPS_RS++;
}

/*------------------------------------------*/
/*--------------Kinect Function-------------*/
/*------------------------------------------*/
/*------------------------------------------ The first Step ------------------------------------------*/
/*Check it already! 3rd*/
void KinectInit()
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return;
		}
		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return;
		}
	}
	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return;
		}
		// Get frame description
		cout << "get color frame description\n" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidthColor);
			pFrameDescription->get_Height(&iHeightColor);
			colorPointCount = iWidthColor * iHeightColor;
			uBufferSizeColor = colorPointCount * 4 * sizeof(BYTE);
			pCSPoints = new CameraSpacePoint[colorPointCount];
			pBufferColor = new BYTE[4 * colorPointCount];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;
		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pFrameReaderColor) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return;
		}
		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}
	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return;
		}
		if (pSensor->get_DepthFrameSource(&pFrameSource) == S_OK)
		{
			pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
			pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
		}
		// Get frame description
		cout << "get depth frame description\n" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidthDepth);
			pFrameDescription->get_Height(&iHeightDepth);
			depthPointCount = iWidthDepth * iHeightDepth;
			pBufferDepth = new UINT16[depthPointCount];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;
		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pFrameReaderDepth) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return;
		}
		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}
	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&Mapper) != S_OK)
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return;
	}
	mDepthImg = cv::Mat::zeros(iHeightDepth, iWidthDepth, CV_16UC1);
	mImg8bit = cv::Mat::zeros(iHeightDepth, iWidthDepth, CV_8UC1);
	mColorImg = cv::Mat::zeros(iHeightColor, iWidthColor, CV_8UC4);
	//dstImage = cv::Mat::zeros(iHeightColor, iWidthColor, CV_8UC4);
	//Mat map_x, map_y;
	////if (!mColorImg.data)
	////{
	////	printf("Can not read jpg file, please make sure the kinect camera is open.\n");
	////	return false;
	////}
	//dstImage.create(mColorImg.size(), mColorImg.type());
	//map_x.create(mColorImg.size(), CV_32FC1);
	//map_y.create(mColorImg.size(), CV_32FC1);
	//for (int i = 0; i < mColorImg.rows; i++){
	//	for (int j = 0; j < mColorImg.cols; j++){
	//		map_x.at<float>(i, j) = mColorImg.cols - j;
	//		map_y.at<float>(i, j) = i;
	//	}
	//}
	//remap(mColorImg, dstImage, map_x, map_y, CV_INTER_LINEAR);
}

/*------------------------------------------ The second Step ------------------------------------------*/
/*读color frame, depth frame,然后mapper,然后就show Image*/
/*Check it already! 4th*/
void KinectUpdate()
{
	/*----------------------------Read color data---------------------------*/
	IColorFrame* pFrameColor = nullptr;
	if (pFrameReaderColor->AcquireLatestFrame(&pFrameColor) == S_OK)
	{
		pFrameColor->CopyConvertedFrameDataToArray(uBufferSizeColor, pBufferColor, ColorImageFormat_Rgba);
		pFrameColor->CopyConvertedFrameDataToArray(uBufferSizeColor, mColorImg.data, ColorImageFormat_Bgra);
		pFrameColor->Release();
		pFrameColor = nullptr;
		{
			FPS++;
			glutPostRedisplay();
		}
	}
	/*----------------------------Read depth data---------------------------*/
	IDepthFrame* pDFrameDepth = nullptr;
	if (pFrameReaderDepth->AcquireLatestFrame(&pDFrameDepth) == S_OK)
	{
		pDFrameDepth->CopyFrameDataToArray(depthPointCount, pBufferDepth);
		pDFrameDepth->CopyFrameDataToArray(depthPointCount, reinterpret_cast<UINT16*>(mDepthImg.data));
		pDFrameDepth->Release();
		pDFrameDepth = nullptr;
	}
	/*--------------Mapper Function (Point Cloud)-------------*/
	Mapper->MapColorFrameToCameraSpace(depthPointCount, pBufferDepth, colorPointCount, pCSPoints);
	//blur(mDepthImg, mDepthImg, Size(3, 3));
	//GaussianBlur(mDepthImg, mDepthImg, Size(3, 3), 0, 0);
	//pBufferDepth = reinterpret_cast<UINT16*>(mDepthImg.data);
	//Mapper->MapColorFrameToCameraSpace(depthPointCount, pBufferDepth, colorPointCount, pCSPoints);
	/*--------------Call Window Function-------------*/
	/*show image can have a color frame to you*/
	ShowImage();
}

void ShowImage()
{
	/*在这里就是读出 color map，其中 depth 的被删掉了*/
	/*open a color map, the depth map was deleted by debby lee*/
	namedWindow("Color Map");
	/*--------------Depth Image-------------*/
	//namedWindow("Depth Map");
	//// Convert from 16bit to 8bit
	//mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
	//cv::imshow("Depth Map", mImg8bit);
	/*--------------Set Mouse Callback Function and Find ROI-------------*/
	/*-----------Use green box to get ROI by mouse-----------*/
	cvSetMouseCallback("Color Map", onMouseROI, NULL);
	//if (ROI.data != NULL)
	//{
	//	ROI.release();
	//}
	mColorImg.copyTo(ROI);
	//dstImage.copyTo(ROI);
	// mColorImg.copyTo(ROI);
	// finish or not finish the ROI getting process
	if (ROI_S2 == TRUE && ROI_S1 == FALSE)
	{
		int thickness = 2;
		rectangle(ROI, ROI_p1, ROI_p2, Scalar(0, 255, 0), thickness);
		/*when the code make sure that the ROI Rec has been done. Use FindROI to do color tracking*/
		FindROI();
	}
	cv::imshow("Color Map", ROI);
}

void onMouseROI(int event, int x, int y, int flags, void* param)
{
	int thickness = 2;
	//Push
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		ROI_rect.x = x;
		ROI_rect.y = y;
		ROI_S1 = TRUE;
		ROI_S2 = FALSE;
	}
	//Release
	else if (event == CV_EVENT_LBUTTONUP)
	{
		ROI_S1 = FALSE;
		ROI_rect.height = y - ROI_rect.y;
		ROI_rect.width = x - ROI_rect.x;
		ROI_p2 = Point(x, y);
	}
	//if (ROI.data != NULL)
	//{
	//	ROI.release();
	//}
	//dstImage.copyTo(ROI);
	mColorImg.copyTo(ROI);
	//Drag
	if (flags == CV_EVENT_FLAG_LBUTTON)
	{
		ROI_S2 = TRUE;
		ROI_p1 = Point(ROI_rect.x, ROI_rect.y);
		ROI_p2 = Point(x, y);
		rectangle(ROI, ROI_p1, ROI_p2, Scalar(0, 255, 0), 2);
		cv::imshow("Color Map", ROI);
	}
}

/*---------------------------------------*/
/*--------------ROI Function-------------*/
/*---------------------------------------*/
void FindROI()
{
	Mat ROI_Image;
	/*--------------Find ROI-------------*/
	if (ROI_p2.x > ROI_p1.x && ROI_p2.y > ROI_p1.y)
	{
		/*get the ROI region that you choose by mouse*/
		//ROI_Image = dstImage.colRange(ROI_p1.x, ROI_p2.x + 1).rowRange(ROI_p1.y, ROI_p2.y + 1).clone();
		ROI_Image = mColorImg.colRange(ROI_p1.x, ROI_p2.x + 1).rowRange(ROI_p1.y, ROI_p2.y + 1).clone();
	}
	else
	{
		OutputDebugString("You push the left button, please add ROI from left-top to right-down, don't mess up with that\n");
		return;
	}
	//namedWindow("ROI");
	//imshow("ROI", ROI_Image);
	/*--------------BGR to YCrCb-------------*/
	Mat ROI_YCrCb;
	cvtColor(ROI_Image, ROI_YCrCb, CV_BGR2YCrCb);
	/*-------------after BGR to YCrCb, the image was storaged in ROI_YCrCb-------------*/
	//namedWindow("YCrCb");
	//imshow("YCrCb", ROI_YCrCb);
	/*-------------- Color Detection and Tracking-------------*/
	int channels = ROI_YCrCb.channels();
	int rows = ROI_YCrCb.rows;
	int cols = ROI_YCrCb.cols;
	int steps = ROI_YCrCb.step;
	ROIcount = 0;
	ROIDepthCount = 0;
	if (ROIPixel != nullptr)
	{
		delete[] ROIPixel;
		ROIPixel = nullptr;
	}
	ROIPixel = new Point2i[1000];
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (ROIcount > 999)
			{
				//cout << endl << "--------Array Out of range--------" << endl;
				OutputDebugString("--------Array Out of range---------//-------Array Out of range--------//--------Array Out of range--------\n");
				exit(1);
			}
			uchar IsRed = 0;
			/*input & output value, OpenCV matrix input & output value function*/
			OutputValue(ROI_YCrCb, i, j, 1, &IsRed);
			//threshold = 150 for fluorescent pink
			//threshold = 170 for red
			if (IsRed > 155)
			{
				ROI_Image.at<Vec4b>(i, j)[0] = 255;
				ROI_Image.at<Vec4b>(i, j)[1] = 0;
				ROI_Image.at<Vec4b>(i, j)[2] = 0;
				InputValue(ROI, 0, i + ROI_p1.y, j + ROI_p1.x, 0, 255);
				InputValue(ROI, 0, i + ROI_p1.y, j + ROI_p1.x, 1, 0);
				InputValue(ROI, 0, i + ROI_p1.y, j + ROI_p1.x, 2, 0);
				ROIPixel[ROIcount].x = j + ROI_p1.x;
				ROIPixel[ROIcount].y = i + ROI_p1.y;
				ROICenterColorS_New.x += ROIPixel[ROIcount].x;
				ROICenterColorS_New.y += ROIPixel[ROIcount].y;
				ROIcount++;
			}
		}
	}
	//imshow("ROI", ROI_Image);
	/*If we got the color we want, do this*/
	if (ROIcount > 0)
	{
		/*get the mean value */
		ROICenterColorS_New.x = static_cast<int>(ROICenterColorS_New.x / ROIcount);
		ROICenterColorS_New.y = static_cast<int>(ROICenterColorS_New.y / ROIcount);
		/*call mapper function*/
		CameraSpaceROI();
		/*tracking ROI, let the probe tip always in the center of ROI*/
		MoveROI();
	}
	else if (ROIcount == 0)
	{
		ROICenterColorS_Old.x = ROICenterColorS_New.x = 0;
		ROICenterColorS_Old.y = ROICenterColorS_New.y = 0;
	}
}

void CameraSpaceROI()
{
	if (ROICameraSP != nullptr)
	{
		delete[] ROICameraSP;
		ROICameraSP = nullptr;
	}
	ROIDepthCount = 0;
	ROICenterCameraS.x = 0;
	ROICenterCameraS.y = 0;
	ROICenterCameraS.z = 0;
	//跑一个 loop，看一下 depth count 有几个
	for (int i = 0; i < ROIcount; i++)
	{
		int index1 = ROIPixel[i].x + ROIPixel[i].y * iWidthColor;
		if (pCSPoints[index1].Z != -1 * numeric_limits<float>::infinity())
		{
			ROIDepthCount++;
		}
	}
	CameraSpacePoint* Temp = new CameraSpacePoint[ROIDepthCount];
	int idx1 = 0;
	for (int i = 0; i < ROIcount; i++)
	{
		int idx2 = ROIPixel[i].x + ROIPixel[i].y * iWidthColor;
		/*what does this mean?*/
		if (pCSPoints[idx2].Z != -1 * numeric_limits<float>::infinity())
		{
			//pCSPoints[idx2].X *= 1;
			Temp[idx1].X = -pCSPoints[idx2].X;
			Temp[idx1].Y = pCSPoints[idx2].Y;
			Temp[idx1].Z = -pCSPoints[idx2].Z;
			//cout << Temp[idx1].X << " " << Temp[idx1].Y << " " << Temp[idx1].Z << endl;
			idx1++;
		}
	}
	/*---------------------------------Bubble sort---------------------------------*/
	for (int i = ROIDepthCount - 1; i > 0; --i)
	{
		for (int j = 0; j < i; ++j)
		{
			if (Temp[j].Z < Temp[j + 1].Z)
			{
				CameraSpacePoint temp;
				temp.X = Temp[j].X;
				temp.Y = Temp[j].Y;
				temp.Z = Temp[j].Z;
				Temp[j].X = Temp[j + 1].X;
				Temp[j].Y = Temp[j + 1].Y;
				Temp[j].Z = Temp[j + 1].Z;
				Temp[j + 1].X = temp.X;
				Temp[j + 1].Y = temp.Y;
				Temp[j + 1].Z = temp.Z;
			}
		}
	}
	//Set int threshold and filtering
	int IndexLim = 0;
	for (int i = 0; i < ROIDepthCount; i++)
	{
		if (Temp[i].Z < Temp[0].Z*1.02)
		{
			IndexLim = i;
			break;
		}
	}
	//Mismatch did not occur
	if (IndexLim == 0)
	{
		ROICameraSP = new CameraSpacePoint[ROIDepthCount];
		for (int i = 0; i < ROIDepthCount; i++)
		{
			ROICameraSP[i].X = Temp[i].X;
			ROICameraSP[i].Y = Temp[i].Y;
			ROICameraSP[i].Z = Temp[i].Z;
			ROICenterCameraS.x += ROICameraSP[i].X;
			ROICenterCameraS.y += ROICameraSP[i].Y;
			ROICenterCameraS.z += ROICameraSP[i].Z;
		}
	}
	//Mismatch occur
	else
	{
		ROICameraSP = new CameraSpacePoint[IndexLim];
		for (int i = 0; i < IndexLim; i++)
		{
			ROICameraSP[i].X = Temp[i].X;
			ROICameraSP[i].Y = Temp[i].Y;
			ROICameraSP[i].Z = Temp[i].Z;
			ROICenterCameraS.x += ROICameraSP[i].X;
			ROICenterCameraS.y += ROICameraSP[i].Y;
			ROICenterCameraS.z += ROICameraSP[i].Z;
		}
		ROIDepthCount = IndexLim;
	}
	delete[] Temp;
	/*------------------------------------------------------------------*/
	if (ROIDepthCount > 0)
	{
		ROICenterCameraS.x = ROICenterCameraS.x / ROIDepthCount;
		ROICenterCameraS.y = ROICenterCameraS.y / ROIDepthCount;
		ROICenterCameraS.z = ROICenterCameraS.z / ROIDepthCount;
	}
	else if (ROIDepthCount == 0)
	{
		ROICenterCameraS.x = 0;
		ROICenterCameraS.y = 0;
		ROICenterCameraS.z = 0;
	}
}

void MoveROI()
{
	if (ROICenterColorS_Old.x == 0 && ROICenterColorS_Old.y == 0)
	{
		ROICenterColorS_Old.x = ROICenterColorS_New.x;
		ROICenterColorS_Old.y = ROICenterColorS_New.y;
	}
	else if (ROICenterColorS_Old.x != 0 && ROICenterColorS_New.y != 0)
	{
		Vec2i Dir;
		//Dir.val[0] = ROICenterColorS_New.x - ROICenterColorS_Old.x;
		//Dir.val[1] = ROICenterColorS_New.y - ROICenterColorS_Old.y;
		Point2i center;
		center.x = (ROI_p1.x + ROI_p2.x) / 2;
		center.y = (ROI_p1.y + ROI_p2.y) / 2;
		Dir.val[0] = ROICenterColorS_New.x - center.x;
		Dir.val[1] = ROICenterColorS_New.y - center.y;
		ROI_p1.x = ROI_p1.x + Dir.val[0];
		ROI_p1.y = ROI_p1.y + Dir.val[1];
		ROI_p2.x = ROI_p2.x + Dir.val[0];
		ROI_p2.y = ROI_p2.y + Dir.val[1];
		ROICenterColorS_Old.x = ROICenterColorS_New.x;
		ROICenterColorS_Old.y = ROICenterColorS_New.y;
		ROICenterColorS_New.x = 0;
		ROICenterColorS_New.y = 0;
	}
}

/*we need to storage the probe tip's coordinate position in the machine coordinate system and the opengl coordinate system*/
void ROICameraSPStorage()
{
	if (ROICameraSP_Storage == nullptr)
	{
		ROICameraSP_Storage = new CameraSpacePoint[StorageSize];
		ROICameraSP_Proj_Storage = new CameraSpacePoint[StorageSize];
		ROICameraSP_MechCoord_Storage = new CameraSpacePoint[StorageSize];
		ROICameraSP_MechCoord_Proj_Storage = new CameraSpacePoint[StorageSize];
		ROIStorageCount = 0;
	}
	//if (IS_KEY_F1_UP && ROIDepthCount > 0)
	//{
	//
	//	if (ROIStorageCount + ROIDepthCount < StorageSize)
	//	{
	//
	//		for (int i = 0; i < ROIDepthCount; i++)
	//		{
	//
	//			//ROICameraSP_Storage[i + ROIStorageCount] = ROICameraSP[i];
	//			ROICameraSP_Storage[i + ROIStorageCount] = ROICameraSP[i];
	//			ROICameraSP_MechCoord_Storage[i + ROIStorageCount] = ROICameraSP_MechCoord[i];
	//			
	//		}
	//
	//		ROIStorageCount += ROIDepthCount;
	//
	//	}
	//	else
	//	{
	//		cout << endl << "The array ROICameraSP_Storage is full, press F2 to clear" << endl;
	//	}
	//}
	if (!ROI_IS_COLLIDE)
	{
		if (ROIStorageCount + 1 < StorageSize)
		{
			CameraSpacePoint Center;
			Center.X = ROICenterCameraS.x;
			Center.Y = ROICenterCameraS.y;
			Center.Z = ROICenterCameraS.z;
			//ROICameraSP_Storage[i + ROIStorageCount] = ROICameraSP[i];
			ROICameraSP_Storage[ROIStorageCount] = Center;
			ROICameraSP_Proj_Storage[ROIStorageCount] = *ARFunc_ROICSP_Proj/*Center*/;
			ROICameraSP_MechCoord_Storage[ROIStorageCount] = *ROICameraSP_MechCoord;
			ROICameraSP_MechCoord_Proj_Storage[ROIStorageCount] = *ROICameraSP_Proj_MechCoord;
			ROIStorageCount += 1;
		}
		else
		{
			cout << endl << "The array ROICameraSP_Storage is full, press F2 to clear" << endl;
		}
	}
	else
	{
		cout << endl << "Collide!! Can not set path here!!!!" << endl;
	}
}

/*--------------------------------------------*/
/*--------------Load Obj Function-------------*/
/*--------------------------------------------*/
void loadOBJModel()
{
	// read an obj model here
	if (OBJ != NULL) {
		free(OBJ);
	}
	//OBJ = glmReadOBJ("box33/box5.obj");
	OBJ = glmReadOBJ("box33/456.obj");
	//printf("%s\n", filename);
	// traverse the color model
	traverseModel();
	//GLuint a = OBJ->numvertices;
	//GLuint b = OBJ->numnormals;
	//GLuint c = OBJ->numgroups;
}

/*It's traverseModel because we need to read the data column by column*/
void traverseModel()
{
	GLMgroup* group = OBJ->groups;
	float maxx, maxy, maxz;
	float minx, miny, minz;
	float dx, dy, dz;
	maxx = minx = OBJ->vertices[3];
	maxy = miny = OBJ->vertices[4];
	maxz = minz = OBJ->vertices[5];
	for (unsigned int i = 2; i <= OBJ->numvertices; i++) {
		GLfloat vx, vy, vz;
		vx = OBJ->vertices[i * 3 + 0];
		vy = OBJ->vertices[i * 3 + 1];
		vz = OBJ->vertices[i * 3 + 2];
		if (vx > maxx) maxx = vx;  if (vx < minx) minx = vx;
		if (vy > maxy) maxy = vy;  if (vy < miny) miny = vy;
		if (vz > maxz) maxz = vz;  if (vz < minz) minz = vz;
	}
	//printf("max\n%f %f, %f %f, %f %f\n", maxx, minx, maxy, miny, maxz, minz);
	dx = maxx - minx;
	dy = maxy - miny;
	dz = maxz - minz;
	//printf("dx,dy,dz = %f %f %f\n", dx, dy, dz);
	GLfloat normalizationScale = myMax(myMax(dx, dy), dz) / 2;
	OBJ->position[0] = (maxx + minx) / 2;
	OBJ->position[1] = (maxy + miny) / 2;
	OBJ->position[2] = (maxz + minz) / 2;
	int gCount = 0;
	while (group) {
		//printf("gCount: %i \n", gCount);
		for (unsigned int i = 0; i < group->numtriangles; i++) {
			//printf("numtriangles: %i \n", i);
			// triangle index
			int triangleID = group->triangles[i];
			//printf("triangle index: %i \n", triangleID);
			// the index of each vertex
			int indv1 = OBJ->triangles[triangleID].vindices[0];
			int indv2 = OBJ->triangles[triangleID].vindices[1];
			int indv3 = OBJ->triangles[triangleID].vindices[2];
			// vertices
			GLfloat vx, vy, vz;
			double scale = 0.001;
			//double scale = 1;
			vx = OBJ->vertices[indv1 * 3];
			vy = OBJ->vertices[indv1 * 3 + 1];
			vz = OBJ->vertices[indv1 * 3 + 2];
			//printf("vertices1 %f %f %f\n", vx, vy, vz);
			/* The model size's unit is mm by glm's size unit is m, so we use a scale to resize it\n*/
			vertices[gCount][i * 9 + 0] = vx * scale;
			vertices[gCount][i * 9 + 1] = vy * scale;
			vertices[gCount][i * 9 + 2] = vz * scale;
			vx = OBJ->vertices[indv2 * 3];
			vy = OBJ->vertices[indv2 * 3 + 1];
			vz = OBJ->vertices[indv2 * 3 + 2];
			//printf("vertices2 %f %f %f\n", vx, vy, vz);
			vertices[gCount][i * 9 + 3] = vx * scale;
			vertices[gCount][i * 9 + 4] = vy * scale;
			vertices[gCount][i * 9 + 5] = vz * scale;
			vx = OBJ->vertices[indv3 * 3];
			vy = OBJ->vertices[indv3 * 3 + 1];
			vz = OBJ->vertices[indv3 * 3 + 2];
			//printf("vertices3 %f %f %f\n", vx, vy, vz); 
			vertices[gCount][i * 9 + 6] = vx * scale;
			vertices[gCount][i * 9 + 7] = vy * scale;
			vertices[gCount][i * 9 + 8] = vz * scale;
			//printf("\n");
			//if (gCount == 2)
			//{
			//	//printf("triangleID %i\n", triangleID);
			//	printf("index %i\n", indv1 * 3);
			//	printf("vertices1 %f %f %f\n", OBJ->vertices[indv1 * 3], OBJ->vertices[indv1 * 3 + 1], OBJ->vertices[indv1 * 3 + 2]);
			//	printf("index %i\n", indv2 * 3);
			//	printf("vertices2 %f %f %f\n", OBJ->vertices[indv2 * 3], OBJ->vertices[indv2 * 3 + 1], OBJ->vertices[indv2 * 3 + 2]);
			//	printf("index %i\n", indv3 * 3);
			//	printf("vertices3 %f %f %f\n", OBJ->vertices[indv3 * 3], OBJ->vertices[indv3 * 3 + 1], OBJ->vertices[indv3 * 3 + 2]);
			//}
			//the index of each normals
			int indn1 = OBJ->triangles[triangleID].nindices[0];
			int indn2 = OBJ->triangles[triangleID].nindices[1];
			int indn3 = OBJ->triangles[triangleID].nindices[2];
			// the vertex normal
			normals[gCount][i * 9 + 0] = OBJ->normals[indn1 * 3 + 0];
			normals[gCount][i * 9 + 1] = OBJ->normals[indn1 * 3 + 1];
			normals[gCount][i * 9 + 2] = OBJ->normals[indn1 * 3 + 2];
			normals[gCount][i * 9 + 3] = OBJ->normals[indn2 * 3 + 0];
			normals[gCount][i * 9 + 4] = OBJ->normals[indn2 * 3 + 1];
			normals[gCount][i * 9 + 5] = OBJ->normals[indn2 * 3 + 2];
			normals[gCount][i * 9 + 6] = OBJ->normals[indn3 * 3 + 0];
			normals[gCount][i * 9 + 7] = OBJ->normals[indn3 * 3 + 1];
			normals[gCount][i * 9 + 8] = OBJ->normals[indn3 * 3 + 2];
			int indt1 = OBJ->triangles[triangleID].tindices[0];
			int indt2 = OBJ->triangles[triangleID].tindices[1];
			int indt3 = OBJ->triangles[triangleID].tindices[2];
			vtextures[gCount][i * 6 + 0] = OBJ->texcoords[indt1 * 2 + 0];
			vtextures[gCount][i * 6 + 1] = OBJ->texcoords[indt1 * 2 + 1];
			vtextures[gCount][i * 6 + 2] = OBJ->texcoords[indt2 * 2 + 0];
			vtextures[gCount][i * 6 + 3] = OBJ->texcoords[indt2 * 2 + 1];
			vtextures[gCount][i * 6 + 4] = OBJ->texcoords[indt3 * 2 + 0];
			vtextures[gCount][i * 6 + 5] = OBJ->texcoords[indt3 * 2 + 1];
			//printf("vtextures: %f %f %f\n", vtextures[gCount][i * 6 + 0], vtextures[gCount][i * 6 + 1], vtextures[gCount][i * 6 + 2]);
			// TODO: texture coordinates should be aligned by yourself
			//OBJ->texcoords[indt1*2];
			//OBJ->texcoords[indt1*2+1];
			//OBJ->texcoords[indt2*2];
			//OBJ->texcoords[indt2*2+1];
			//OBJ->texcoords[indt3*2];
			//OBJ->texcoords[indt3*2+1];
		}
		group = group->next;
		gCount++;
		//vertices;
	}
}

void SetTexObj(char *filename, int ii)
{
	glBindTexture(GL_TEXTURE_2D, /*textures[ii]*/ii);
	IplImage *imageCV; // 影像的資料結構
	imageCV = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR); // 讀取影像的
														  //cvShowImage("HelloWorld", imageCV);
														  //cvWaitKey(0); // 停留視窗
	char *imageGL;
	//imageGL = (unsigned char *)malloc(imageCV->nChannels * imageCV->height*imageCV->width *sizeof(unsigned char *));
	imageGL = new char[3 * imageCV->height*imageCV->width];
	int step = imageCV->width * imageCV->nChannels;
	for (int i = 0; i < imageCV->height; i++)
	{
		for (int j = 0; j < imageCV->widthStep; j = j + 3)
		{
			//imageGL[(imageCV->height - 1 - i)*imageCV->widthStep + j + 0] = imageCV->imageData[i*imageCV->widthStep + j + 2];
			//imageGL[(imageCV->height - 1 - i)*imageCV->widthStep + j + 1] = imageCV->imageData[i*imageCV->widthStep + j + 1];
			//imageGL[(imageCV->height - 1 - i)*imageCV->widthStep + j + 2] = imageCV->imageData[i*imageCV->widthStep + j + 0];
			imageGL[(imageCV->height - 1 - i)*step + j + 0] = imageCV->imageData[i*step + j + 2];
			imageGL[(imageCV->height - 1 - i)*step + j + 1] = imageCV->imageData[i*step + j + 1];
			imageGL[(imageCV->height - 1 - i)*step + j + 2] = imageCV->imageData[i*step + j + 0];
			//imageGL[i*imageCV->widthStep + j + 0] = imageCV->imageData[i*imageCV->widthStep + j + 2];
			//imageGL[i*imageCV->widthStep + j + 1] = imageCV->imageData[i*imageCV->widthStep + j + 1];
			//imageGL[i*imageCV->widthStep + j + 2] = imageCV->imageData[i*imageCV->widthStep + j + 0];
			//cout << (int)(imageGL[i*imageCV->widthStep + j + 0]) << " " << (int)(imageGL[i*imageCV->widthStep + j + 1]) << " " << (int)(imageGL[i*imageCV->widthStep + j + 2]) << endl;
			//cout << endl << endl;
		}
		//cout << endl << endl;
	}
	//cout << (int)(imageGL[0]) << " " << (int)(imageGL[1]) << " " << (int)(imageGL[2]) << " " << endl;
	//材質控制
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageCV->width, imageCV->height, 0, GL_RGB, GL_UNSIGNED_BYTE, imageGL);
	delete[] imageGL;
	//cvDestroyWindow("HelloWorld"); // 銷毀視窗
	cvReleaseImage(&imageCV); // 釋放IplImage資料結構
							  //使用多材質
							  /*glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
							  gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);*/
}

void Texture()
{
	//glEnable(GL_TEXTURE_2D);
	glGenTextures(7, textures);
	//glPrioritizeTextures(4, textures, priorities); //1最重要 0最不重要 會先被踢掉
	//SetTexObj("box33/box5/Mosaic_Hexagonal_Tile.jpg", textures[0]);
	//SetTexObj("box33/box5/Metal_Steel_Textured_White.jpg", textures[1]);
	//SetTexObj("box33/box5/Wood_Floor_Light.jpg", textures[2]);
	//SetTexObj("box33/box5/Stone_Marble.jpg", textures[3]);
	//SetTexObj("box33/box5/Stone_Brushed_Khaki.jpg", textures[4]);
	//SetTexObj("box33/box5/Wood_Floor_Dark.jpg", textures[5]);
	SetTexObj("box33/box5/m1.jpg", textures[0]);
	SetTexObj("box33/box5/m2.jpg", textures[2]);
	SetTexObj("box33/box5/m3.jpg", textures[1]);
	SetTexObj("box33/box5/m4.jpg", textures[3]);
	SetTexObj("box33/box5/m5.jpg", textures[4]);
	SetTexObj("box33/box5/m6.jpg", textures[5]);
	SetTexObj("box33/box5/m7.jpg", textures[6]);
}

/*---------------------------------------------------*/
/*----------------AR Function Declare----------------*/
/*---------------------------------------------------*/
void ARFunc_FindProj()
{
	/*-----------------------------------*/
	/*改成OBJ頂面名面方程式只要找一次就好*/
	/*-----------------------------------*/
	CameraSpacePoint* CubeTop = new CameraSpacePoint[4];
	//input .obj file's top surface's 4 points
	CubeTop[0].X = OBJ->vertices[3 + 0] * 0.001;
	CubeTop[0].Y = OBJ->vertices[3 + 1] * 0.001;
	CubeTop[0].Z = OBJ->vertices[3 + 2] * 0.001;
	CubeTop[1].X = OBJ->vertices[96 + 0] * 0.001;
	CubeTop[1].Y = OBJ->vertices[96 + 1] * 0.001;
	CubeTop[1].Z = OBJ->vertices[96 + 2] * 0.001;
	CubeTop[2].X = OBJ->vertices[132 + 0] * 0.001;
	CubeTop[2].Y = OBJ->vertices[132 + 1] * 0.001;
	CubeTop[2].Z = OBJ->vertices[132 + 2] * 0.001;
	CubeTop[3].X = OBJ->vertices[12 + 0] * 0.001;
	CubeTop[3].Y = OBJ->vertices[12 + 1] * 0.001;
	CubeTop[3].Z = OBJ->vertices[12 + 2] * 0.001;
	CameraSpacePoint* CubeNorm = new CameraSpacePoint[2];
	//The normal vector of one of the triangles in the plane
	CubeNorm[0].X = OBJ->normals[OBJ->triangles[145].nindices[0] * 3 + 0];
	CubeNorm[0].Y = OBJ->normals[OBJ->triangles[145].nindices[0] * 3 + 1];
	CubeNorm[0].Z = OBJ->normals[OBJ->triangles[145].nindices[0] * 3 + 2];
	//start point & finish point
	CubeNorm[1].X = 0;
	CubeNorm[1].Y = 0;
	CubeNorm[1].Z = 0;
	GLfloat TransM_AR[16] = { 0 };
	TransM_AR[0] = TransM_AR[5] = TransM_AR[10] = TransM_AR[15] = 1;
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(ObjPosi.x, ObjPosi.y, ObjPosi.z);
	glTranslatef(Intersect.X, Intersect.Y, Intersect.Z);
	glMultMatrixf(M_Cubic);
	//glRotatef(-90, 0, 0, 1);
	//glRotatef(180, 0, 0, 1);
	glGetFloatv(GL_MODELVIEW_MATRIX, TransM_AR);
	glPopMatrix();
	ROITrans(CubeNorm, 2, TransM_AR, CubeNorm);
	ROITrans(CubeTop, 4, TransM_AR, CubeTop);
	//figure out the plane equation of top surface of .OBJ file
	ARFuncNormal[0] = CubeNorm[0].X - CubeNorm[1].X;
	ARFuncNormal[1] = CubeNorm[0].Y - CubeNorm[1].Y;
	ARFuncNormal[2] = CubeNorm[0].Z - CubeNorm[1].Z;
	ARFuncNormal[3] = -(ARFuncNormal[0] * CubeTop[0].X + ARFuncNormal[1] * CubeTop[0].Y + ARFuncNormal[2] * CubeTop[0].Z);
	//Figure out the ROI center point to .OBJ file's projection point
	CameraSpacePoint* ROICenter = new CameraSpacePoint;
	ARFunc_ROICSP_Proj = new CameraSpacePoint;
	ROICenter->X = ROICenterCameraS.x;
	ROICenter->Y = ROICenterCameraS.y;
	ROICenter->Z = ROICenterCameraS.z;
	PathGenProjToPlane(ROICenter, 1, ARFuncNormal, ARFunc_ROICSP_Proj);
	//ARFunc_ROICSP_Proj->Y = -ARFunc_ROICSP_Proj->Y;
	GLMgroup* group = OBJ->groups;
	group = group->next;
	group = group->next;
	for (int TriCount = 0; TriCount < group->numtriangles; TriCount++)
	{
		int TriID = group->triangles[TriCount];
		CameraSpacePoint TriVertex[3];
		TriVertex[0].X = OBJ->vertices[OBJ->triangles[TriID].vindices[0] * 3 + 0] * 0.001;
		TriVertex[0].Y = OBJ->vertices[OBJ->triangles[TriID].vindices[0] * 3 + 1] * 0.001;
		TriVertex[0].Z = OBJ->vertices[OBJ->triangles[TriID].vindices[0] * 3 + 2] * 0.001;
		TriVertex[1].X = OBJ->vertices[OBJ->triangles[TriID].vindices[1] * 3 + 0] * 0.001;
		TriVertex[1].Y = OBJ->vertices[OBJ->triangles[TriID].vindices[1] * 3 + 1] * 0.001;
		TriVertex[1].Z = OBJ->vertices[OBJ->triangles[TriID].vindices[1] * 3 + 2] * 0.001;
		TriVertex[2].X = OBJ->vertices[OBJ->triangles[TriID].vindices[2] * 3 + 0] * 0.001;
		TriVertex[2].Y = OBJ->vertices[OBJ->triangles[TriID].vindices[2] * 3 + 1] * 0.001;
		TriVertex[2].Z = OBJ->vertices[OBJ->triangles[TriID].vindices[2] * 3 + 2] * 0.001;
		ROITrans(TriVertex, 3, TransM_AR, TriVertex);
		if (ARFunc_InsideTriCheck(ARFunc_ROICSP_Proj, &TriVertex[0], &TriVertex[2], &TriVertex[1]))
		{
			IS_ARFunc_InsideTriCheck = TRUE;
			break;
		}
		else
		{
			IS_ARFunc_InsideTriCheck = FALSE;
		}
	}
}

/*Check out if the projection point is in the operation area*/
bool ARFunc_InsideTriCheck(CameraSpacePoint* Point, CameraSpacePoint* TriVertex0, CameraSpacePoint* TriVertex1, CameraSpacePoint* TriVertex2)
{
	BOOL IS_INSIDE_TRI = FALSE;
	//Vector AP AB AC in order
	float Vector[3][3];
	Vector[0][0] = Point->X - TriVertex0->X;
	Vector[0][1] = Point->Y - TriVertex0->Y;
	Vector[0][2] = Point->Z - TriVertex0->Z;
	Vector[1][0] = TriVertex1->X - TriVertex0->X;
	Vector[1][1] = TriVertex1->Y - TriVertex0->Y;
	Vector[1][2] = TriVertex1->Z - TriVertex0->Z;
	Vector[2][0] = TriVertex2->X - TriVertex0->X;
	Vector[2][1] = TriVertex2->Y - TriVertex0->Y;
	Vector[2][2] = TriVertex2->Z - TriVertex0->Z;
	float delt = Determinant(Dot(Vector[2], Vector[2]), Dot(Vector[1], Vector[2]), Dot(Vector[1], Vector[2]), Dot(Vector[1], Vector[1]));
	float deltX = Determinant(Dot(Vector[0], Vector[2]), Dot(Vector[1], Vector[2]), Dot(Vector[0], Vector[1]), Dot(Vector[1], Vector[1]));
	float deltY = Determinant(Dot(Vector[2], Vector[2]), Dot(Vector[0], Vector[2]), Dot(Vector[1], Vector[2]), Dot(Vector[0], Vector[1]));
	float u = deltX / delt, v = deltY / delt;
	if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && u + v <= 1)
	{
		IS_INSIDE_TRI = TRUE;
	}
	else
	{
		IS_INSIDE_TRI = FALSE;
	}
	return IS_INSIDE_TRI;
}

/*------------------------------------*/
/*--------------main loop-------------*/
/*------------------------------------*/
int main(int argc, char** argv)
{
	/*--------------initial part--------------*/
	KinectInit(); //Kinect
	GLInit(); //OpenGL
	InitPortAudio();//audio
	MtInit();//dispenser machine
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(iWidthColor, iHeightColor);
	glutCreateWindow("Perspective Projection");
	glutFullScreen();
	glutSpecialFunc(SpecialKeys);
	glutKeyboardFunc(Keyboard);
	glutDisplayFunc(RenderScene);
	//update
	glutIdleFunc(KinectUpdate);
	Texture();
	loadOBJModel();
	glutTimerFunc(1000, timer, 0);
	//glutTimerFunc(125, timer, 1);
	BuildPopupMenu();
	//typedef BOOL(APIENTRY *PFNWGLSWAPINTERVALFARPROC)(int);
	//PFNWGLSWAPINTERVALFARPROC wglSwapIntervalEXT = 0;
	//wglSwapIntervalEXT = (PFNWGLSWAPINTERVALFARPROC)wglGetProcAddress("wglSwapIntervalEXT");
	//wglSwapIntervalEXT(1);
	glutMainLoop();
	if (IS_PORTAUDIO_INIT)
	{
		sine.close();
	}
	cout << "Close I/O = " << MtClose() << endl;
	cout << "End process" << endl;
	return 0;
}