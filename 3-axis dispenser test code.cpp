#include <windows.h>
#include <iostream>
//#include <Source.h>
#include "dllmain.h"
//#include <afxwin.h>
using namespace std;
//using namespace Mt232;
//extern "C" _declspec(dllimport) void MtTestDebby(bool* DBLflag);
//typedef int (*MtTestDebbyFunc)(int a, int b);
//typedef long (*MyMtConnect)(char* ptr);
//typedef long (*MyMtClose)(void);
//typedef long(*MyMtReflash)(struct MotionData *m);
//
//struct MotionData {
//	double x, y, z, u, v, w;
//	double laser;
//	long   out, status, length, in;
//	long   flowNumber;
//	long   wdt;
//	long   cmdid;
//};
//__declspec(dllexport)  long MtConnect(long ptr);
#using <System.dll>
using namespace System;
using namespace System::IO::Ports;
using namespace System::ComponentModel;
bool MtHome(void);

void MtReflashWait(void);

Rs232MotionData* md = new struct Rs232MotionData;

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
	//MtCmd("mt_go_home 50,50,10,50,2,2,1,255"); // 回home的速度(速度x, 速度y, 速度z, 速度u, 順序x, 順序y, 順序z, 順序u, 255 = u不回home)
	MtCmd("mt_go_home 50,50,10,50,2,2,1,3"); // 回home的速度(速度x, 速度y, 速度z, 速度u, 順序x, 順序y, 順序z, 順序u, 255 = u不回home)
	Sleep(1000);
	do {
		MtReflashWait();
		if (MtFlag(MotionStatus::emg_signal))
		{
			MtCmd("mt_abort_home");
			return false;
		}
	} while (MtFlag(MotionStatus::home_x) || MtFlag(MotionStatus::home_y) || MtFlag(MotionStatus::home_z)); //home_x 回到原點的過程 = true
	MtCmd("mt_home_finish");
	MtCmd("mt_speed1 30");
	MtCmd("mt_soft_limit 0,-2,300"); //Axe no., Left limit, Right limit
	MtCmd("mt_soft_limit 1,-2,300");
	MtCmd("mt_soft_limit 2,-2,100");
	MtCmd("mt_soft_limit 3,-450,450");
	MtCmd("mt_check_ot 1,1,1,0"); //enable limit (by axe no)
	MtCmd("mt_out 11,1"); //door switch
	MtCmd("mt_m_acc 150");  //door switch
	MtCmd("mt_v_acc 80");  //door switch
	return true;
}

int main()
{
	//#pragma region CallingDLLFromC
	//
	//	HMODULE hDll = LoadLibrary("ConsoleApplication1.dll");
	//
	//	if (hDll != NULL)
	//	{
	//		DBL = (MtTestDebbyFunc)GetProcAddress(hDll, "MtTestDebby");
	//
	//		mtConnect = (MyMtConnect)GetProcAddress(hDll, "MtConnect");
	//		//mtReflash = (MyMtReflash)GetProcAddress(hDll, "MtReflash");
	//		mtClose = (MyMtClose)GetProcAddress(hDll, "MtClose");
	//
	//		motionComClose = (MyMotionComClose)GetProcAddress(hDll, "MotionComClose");
	//		motionComStatus = (MyMotionComStatus)GetProcAddress(hDll, "MotionComStatus");
	//		motionComSignal = (MyMotionComSignal)GetProcAddress(hDll, "MotionComSignal");
	//		motionFlag = (MyMotionFlag)GetProcAddress(hDll, "MotionFlag");
	//		motionInput = (MyMotionInput)GetProcAddress(hDll, "MotionInput");
	//		motionCmd = (MyMotionCmd)GetProcAddress(hDll, "MotionCmd");
	//
	//		mtRs232Baud = (MyMtRs232Baud)GetProcAddress(hDll, "MtRs232Baud");
	//		mtStart = (MyMtStart)GetProcAddress(hDll, "MtStart");
	//		mtFlag = (MyMtFlag)GetProcAddress(hDll, "MtFlag");
	//		mtInput = (MyMtInput)GetProcAddress(hDll, "MtInput");
	//		mtSize = (MyMtSize)GetProcAddress(hDll, "MtSize");
	//		mtHead = (MyMtHead)GetProcAddress(hDll, "MtHead");
	//		mtTail = (MyMtTail)GetProcAddress(hDll, "MtTail");
	//		mtCmd = (MyMtCmd)GetProcAddress(hDll, "MtCmd");
	//		mtEmpty = (MyMtEmpty)GetProcAddress(hDll, "MtEmpty");
	//		mtThread = (MyMtThread)GetProcAddress(hDll, "MtThread");
	//		mtCheckSum = (MyMtCheckSum)GetProcAddress(hDll, "MtCheckSum");
	//		mtCheckWdt = (MyMtCheckWdt)GetProcAddress(hDll, "MtCheckWdt");
	//		mtError = (MyMtError)GetProcAddress(hDll, "MtError");
	//		mtLaserCmd = (MyMtLaserCmd)GetProcAddress(hDll, "MtLaserCmd");
	//		mtReceived = (MyMtReceived)GetProcAddress(hDll, "MtReceived");
	//
	//		remoteWDT = (MyRemoteWDT)GetProcAddress(hDll, "RemoteWDT");
	//
	//		if (DBL == NULL)
	//		{
	//			cout << "Import MtTestDebby Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//
	//		if (mtConnect == NULL)
	//		{
	//			cout << "Import MtConnect Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		//if (mtReflash == NULL)
	//		//{
	//		//	cout << "Import MtReflash Function Faild" << endl;
	//		//	system("pause");
	//		//	return 0;
	//		//}
	//		if (mtClose == NULL)
	//		{
	//			cout << "Import MtClose Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//
	//		if (motionComClose == NULL)
	//		{
	//			cout << "Import MotionComClose Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionComStatus == NULL)
	//		{
	//			cout << "Import MotionComStatus Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionComSignal == NULL)
	//		{
	//			cout << "Import MotionFlag Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionFlag == NULL)
	//		{
	//			cout << "Import MtClose Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionInput == NULL)
	//		{
	//			cout << "Import MotionInput Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//
	//		if (motionCmd == NULL)
	//		{
	//			cout << "Import MotionCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//
	//		if (mtRs232Baud == NULL)
	//		{
	//			cout << "Import MtRs232Baud Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtStart == NULL)
	//		{
	//			cout << "Import MtStart Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtFlag == NULL)
	//		{
	//			cout << "Import MtFlag Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtInput == NULL)
	//		{
	//			cout << "Import MtInput Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtSize == NULL)
	//		{
	//			cout << "Import MtSize Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtHead == NULL)
	//		{
	//			cout << "Import MtHead Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtTail == NULL)
	//		{
	//			cout << "Import MtTail Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtCmd == NULL)
	//		{
	//			cout << "Import MtCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtEmpty == NULL)
	//		{
	//			cout << "Import MtEmpty Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtThread == NULL)
	//		{
	//			cout << "Import MtThread Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtCheckSum == NULL)
	//		{
	//			cout << "Import MtCheckSum Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtCheckWdt == NULL)
	//		{
	//			cout << "Import MtCheckWdt Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtError == NULL)
	//		{
	//			cout << "Import MtError Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtLaserCmd == NULL)
	//		{
	//			cout << "Import MtLaserCmd Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (mtReceived == NULL)
	//		{
	//			cout << "Import MtReceived Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//		if (remoteWDT == NULL)
	//		{
	//			cout << "Import RemoteWDT Function Faild" << endl;
	//			system("pause");
	//			return 0;
	//		}
	//	}
	//	else
	//	{
	//		cout << "Load library: ConsoleApplication1.dll faild" << endl;
	//		system("pause");
	//		return 0;
	//	}
	//
	//#pragma endregion CallingDLLFromC
	array<String^>^ serialPorts = nullptr;
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
	for each(String^ port in serialPorts)
	{
		Console::WriteLine(port);
	}
	//cout << MtTestDebby(2, 3) << endl;
	char ptr[5] = "COM4";
	cout << "Connect I/O = " << MtConnect(ptr) << endl;
	//Sleep(100);
	//cout << "Connect I/O = " << mtconnect(ptr) << endl;
	//if (mtreflash != NULL)
	//{
	//	if (mtconnect(ptr) != 0)
	//	{
	//		cout << "Connect I/O = " << mtconnect(ptr) << endl;
	//		Sleep(100);
	//		
	//		cout << "Reflash I/O = " << mtreflash(md) << endl;
	//	}
	//}
	//cout << mtReflash(md) << endl;
	//md = new struct Rs232MotionData;
	//Rs232MotionData* md = new struct Rs232MotionData;
	//md->x = 0;
	cout << "MtReflash: " << MtReflash(md) << endl;
	cout << "md->x : " << md->x << endl;
	cout << "mdr.wdt : " << mdr.wdt << endl;
	//MtCmd("mt_emg 0");
	bool IS_HOME = MtHome();
	Sleep(100);
	for (int i = 0; i < 5; i++)
	{
		MtCmd("mt_out 12,1");
		Sleep(100);
		MtCmd("mt_out 12,0");
		Sleep(100);
	}
	//MtCmd("mt_speed 100");
	//MtCmd("mt_mr_x 30"); //mr = 相對座標; m = 絕對座標
	//MtCmd("mt_mr_y 30"); //mr = 相對座標; m = 絕對座標
	//Sleep(100);
	MtCmd("mt_m_x 45"); //mr = 相對座標; m = 絕對座標
	MtCmd("mt_m_y 24"); //mr = 相對座標; m = 絕對座標
	Sleep(100);
	MtReflash(md);
	cout << "md->x : " << md->x << endl;
	cout << "Close I/O = " << MtClose() << endl;
	//FreeLibrary(hDll);
	cout << "End process" << endl;
	system("pause");
	return 0;
}