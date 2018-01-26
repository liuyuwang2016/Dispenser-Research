//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~佛祖保佑，永无Bug~~~~~~~~~~~~~
#include "Declae.h"

#pragma region Init
void init()
{
	KinectInit();     //Kinect
	GLInit();         //OpenGL
	InitPortAudio();  //audio
	MtInit();         //dispenser machine
}
#pragma endregion Init

#pragma region SpecialKeys
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
	else if (key == GLUT_KEY_F2)
	{
		if (ROICameraSP_Storage != nullptr)
		{
			delete[] ROICameraSP_Storage;
		}
		if (ROICameraSP_Proj_Storage != nullptr)
		{
			delete[] ROICameraSP_Proj_Storage;
		}
		if (ROICameraSP_MachineCoord_Storage != nullptr)
		{
			delete[] ROICameraSP_MachineCoord_Storage;
		}
		if (ROICameraSP_MachineCoord_Proj_Storage != nullptr)
		{
			delete[] ROICameraSP_MachineCoord_Proj_Storage;
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
	glutPostRedisplay();
}
#pragma endregion SpecialKeys

#pragma region RenderScene
void DrawProbeTip()
{
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
}

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
			DrawMeshes();
			break;
		case 2:
			DrawPointCloud();
			break;
		case 3:
			Draw3DLine();
			break;
		case 4:
			Draw3DPlane();
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
	DrawProbeTip();
	/*--------------Draw Probe Tip-------------*/
	//if (ROIDepthCount != 0)
	//{
	//	glPushMatrix();
	//	glPointSize(2.0f);
	//	glBegin(GL_POINTS);
	//	for (int i = 0; i < ROIDepthCount; i++)
	//	{
	//		glColor3ub(0, 255, 0);
	//		/*draw probe tip's depth points*/
	//		glVertex3f(ROICameraSP[i].X, ROICameraSP[i].Y, ROICameraSP[i].Z);
	//	}
	//	glEnd();
	//	glPopMatrix();
	//}
	/*--------------Want to draw Plane-------------*/
	if (Planedepthcount != 0)
	{
		glPushMatrix();
		glPointSize(2.0f);
		glBegin(GL_POINTS);
		CameraSpacePoint* temp = new CameraSpacePoint[Planedepthcount];
		int index4 = 0;
		for (int i = 0; i < PlanePixelcount; i++)
		{
			glColor3ub(0, 127, 255);
			int index5 = PlanePixel[i].x + PlanePixel[i].y * iWidthColor;
			if (pCSPoints[index5].Z != -1 * numeric_limits<float>::infinity())
			{
				temp[index4].X = pCSPoints[index5].X;
				temp[index4].Y = pCSPoints[index5].Y;
				temp[index4].Z = -pCSPoints[index5].Z;
				glVertex3f(temp[index4].X, temp[index4].Y, temp[index4].Z);
				index4++;
			}
			cout << "画出来了" << index4 << "个" << endl;
		}
		delete[] temp;
		glEnd();
		glPopMatrix();

	}
	/*--------------------碰撞侦测-------------------*/
	/*--------------Collision Detection-------------*/
	if (ROIDepthCount != 0 && ARFunc_IS_ON)
	{
		GLfloat TransM[16] = { 0 };
		TransM[0] = TransM[5] = TransM[10] = TransM[15] = 1;
		glPushMatrix();
		glLoadIdentity();
		//移动到原点d
		glMultMatrixf(M_Cubic_inv);
		//把笔的坐标移动到机器坐标原点
		glTranslatef(-Intersect.X, -Intersect.Y, -Intersect.Z);
		glTranslatef(-ObjPosi.x, -ObjPosi.y, -ObjPosi.z);
		//http://blog.csdn.net/lyx2007825/article/details/8792475
		//http://blog.csdn.net/hippig/article/details/7894034
		glGetFloatv(GL_MODELVIEW_MATRIX, TransM);
		glPopMatrix();

		if (ROICameraSP_TouchDetec != nullptr)
		{
			delete[] ROICameraSP_TouchDetec;
		}
		ROICameraSP_TouchDetec = new CameraSpacePoint[ROIDepthCount];

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

					if (Psi*180.0f / CV_PI > 90.0f)
					{
						switch (gCount)
						{
						//这里的 case3 的意思是 gCount 等于 3,也就是说原来代码中的第三个group是圆圈。
						case 3:
							/*the circle of the obj*/
							CollideCount_Circle++;
							break;
							//上面的所有case都不为真
						default:
							CollideCount++;//等于46
							break;
						}
					}
				}
				//gCount = 8
				gCount++;
				group = group->next;
			}

			if (CollideCount_Circle == 0)
			{
				ROI_IS_COLLIDE = FALSE;
			}//在这里被写死了，当collidecount达到这个数值的时候就说明确实碰撞了
			//numtriangle = 168
			else if (CollideCount == OBJ->numtriangles - 76)
			{
				ROI_IS_COLLIDE = TRUE;
				break;
			}
			CollideCount = 0;
			CollideCount_Circle = 0;
		}
	}

	/*--------------AR Function-------------*/
	/*find depth points and do not collide, call ARFunc_FindProj()，
	 *如果没有发生碰撞，那么就执行AR程式*/
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
			float Red = 0, Green = 0, Blue = 0;
			/*change the color of the line by the distance range*/
			if (dist > high_range)
			{
				Red = 0;
				Green = 1;
				Blue = 0;
			}
			else if (dist < low_range)
			{
				Red = 1;
				Green = 0;
				Blue = 0;
			}
			else
			{
				Red = (dist - low_range) / ((low_range + high_range) - low_range) * (high_r - low_r) + low_r;
				Green = (dist - (low_range + high_range)) / (high_range - (low_range + high_range)) * (high_g - low_g) + low_g;
				Blue = -1.0f / pow((high_range - low_range) / 2, 2) * pow((dist - (low_range + high_range) / 2), 2) + 1.0f;
			}

			/* draw line */
			glEnable(GL_LINE_STIPPLE);
			glPushMatrix();
			glColor3f(Red, Green, Blue);
			glLineWidth(2.5);
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

	/*--------------Transfer Prob Tip from GL Coord. to Machine Coord.-------------*/
	if (ROIDepthCount > 0 && ROICameraSP != nullptr && !ROI_IS_COLLIDE)
	{
		GLfloat TransM_toMechCoord[16] = { 0 };
		TransM_toMechCoord[0] = TransM_toMechCoord[5] = TransM_toMechCoord[10] = TransM_toMechCoord[15] = 1;
		glPushMatrix();
		glLoadIdentity();
		glTranslatef(TipPosi.x, TipPosi.y, TipPosi.z);
		//http://cuiqingcai.com/1658.html 旋转示例
		glRotatef(-90, 1, 0, 0);
		//glMultMatrixf(const GLfloat *m);//把m指定的16个值作为一个矩阵，与当前矩阵相乘，并把结果存储在当前矩阵中 
		//glMultMatrix 假设当前矩阵是C那么用矩阵M 调用glMultMatrix 对顶点v的变换就从原来的C*v变成C * M * v
		//http://blog.csdn.net/mathgeophysics/article/details/11434345
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
	/*--------------Calculate deviation & Draw Cubic 方块也要跟着移动的距离-------------*/
	CameraSpacePoint* Y_Devia = new CameraSpacePoint[2];
	Y_Devia[0].X = Y_Devia[0].Y = Y_Devia[0].Z = 0;
	Y_Devia[1].X = Y_Devia[1].Y = Y_Devia[1].Z = 0;

	if (ROIStorageCount > 0 && CUBIC_MOVE)
	{
		float dy = 0;
		switch (STORAGE_TYPE)
		{
		case 0:
			dy = ROICameraSP_MachineCoord_Storage[mtmove_step].Y;
			break;
		case 1:
			dy = ROICameraSP_MachineCoord_Proj_Storage[mtmove_step].Y;
			break;
		}
		GLfloat TransM_OBJ[16] = { 0 };
		TransM_OBJ[0] = TransM_OBJ[5] = TransM_OBJ[10] = TransM_OBJ[15] = 1;

		glPushMatrix();
		glLoadIdentity();
		glTranslatef(Intersect.X, Intersect.Y, Intersect.Z);
		glMultMatrixf(M_Cubic);

		glRotatef(90, 1, 0, 0);
		glTranslatef(TipPosi.x, TipPosi.y, TipPosi.z);
		glGetFloatv(GL_MODELVIEW_MATRIX, TransM_OBJ);
		glPopMatrix();

		Y_Devia[1].Y = dy;

		ROITrans(Y_Devia, 2, TransM_OBJ, Y_Devia);

		DeviaDueToY->X = Y_Devia[1].X - Y_Devia[0].X;
		DeviaDueToY->Y = Y_Devia[1].Y - Y_Devia[0].Y + (Y_Devia[1].Z - Y_Devia[0].Z)*tan(-dev_theta);
		DeviaDueToY->Z = Y_Devia[1].Z - Y_Devia[0].Z;
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

	DrawCubic();
	//ARFunc_IS_ON = FALSE;
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
	/*glPrintf is used to draw the probe tip's coordinate position*/
	if (ROICameraSP_MechCoord != nullptr)
	{
		glPrintf(">>glut(%i, %i)", ROICenterColorS_Old.x - 60, ROICenterColorS_Old.y - 35);
		glPrintf("(%.3f, %.3f, %.3f)", ROICameraSP_MechCoord[0].X - 0.172, ROICameraSP_MechCoord[0].Y - 0.158, ROICameraSP_MechCoord[0].Z + 0.576);
	}
	glPopMatrix();
	if (ARFunc_IS_ON)
	{
		/*--------------Change Color of projection line-------------*/
		float dist = ARFunc_ROICSP_Proj_Dist * 1000;
		const int low_range = 30, high_range = 100;
		int low_r = 1, high_r = 0, low_g = 0, high_g = 1;
		float Red = 0, Green = 0, Blue = 0;
		if (dist > high_range)
		{
			Red = 0;
			Green = 1;
			Blue = 0;
		}
		else if (dist < low_range)
		{
			Red = 1;
			Green = 0;
			Blue = 0;
		}
		else
		{
			Red = (dist - low_range) / ((low_range + high_range) - low_range) * (high_r - low_r) + low_r;
			Green = (dist - (low_range + high_range)) / (high_range - (low_range + high_range)) * (high_g - low_g) + low_g;
			Blue = -1.0f / pow((high_range - low_range) / 2, 2) * pow((dist - (low_range + high_range) / 2), 2) + 1.0f;
		}
		/*--------------Show Prob Tip Coordinate. on Screen-------------*/
		glPushMatrix();
		glColor3f(Red, Green, Blue);
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
#pragma endregion RenderScene



int main(int argc, char** argv)
{
	/*--------------initial part--------------*/
	init();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(iWidthColor, iHeightColor);
	glutCreateWindow("Perspective Projection");
	glutFullScreen();
	glutSpecialFunc(SpecialKeys);
	glutKeyboardFunc(Keyboard);
	glutDisplayFunc(RenderScene);
	glutIdleFunc(KinectUpdate);
	Texture();
	loadOBJModel();
	/*--------------above this do not need check again--------------*/

	//update
	glutTimerFunc(1000, timer, 0);
	//glutTimerFunc(125, timer, 1);
	BuildPopupMenu();
	glutMainLoop();
	if (IS_PORTAUDIO_INIT)
	{
		sine.close();
	}
	//cout << "Let the machine go home" << MtHome() << endl;
	cout << "Close I/O = " << MtClose() << endl;
	cout << "End process" << endl;
	return 0;
}