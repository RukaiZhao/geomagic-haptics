#define _WINSOCK_DEPRECATED_NO_WARNINGS

#pragma comment(lib,"ws2_32.lib")
#include <WinSock2.h>
#include <string>
#include <iostream>
#include <time.h>
#include <process.h>

#include <GL/GL.h>
#include <GL/glut.h>

#include <HL/hl.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

SOCKET Connection;

hduVector3Dd global_position;

struct PointMass
{
    hduVector3Dd m_position;
    hduVector3Dd m_velocity;
    HDdouble m_mass; 
    HDdouble m_kStiffness;
    HDdouble m_kDamping;
};

//sending current cursor position to the server
unsigned int _stdcall send_position(void* data){
	char send_buffer [1024];
    

	while(true){
	int j;
	 memset(send_buffer, 0, sizeof(send_buffer));
	 j = sprintf_s(send_buffer,1024,"%f\n",global_position[0]);
	 j += sprintf_s(send_buffer+j,1024-j,"%f\n",global_position[1]);
	 j += sprintf_s(send_buffer+j,1024-j,"%f\n",global_position[2]);
		
	 send(Connection, send_buffer, sizeof(send_buffer), NULL);
	 
	}

	return 0;
}

//rendering haptic force
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata)
{
    char rec_buffer[1024];
	float result_x, result_y, result_z=0.0;
	
	PointMass *pPointMass = static_cast<PointMass *>(userdata);
	hlCacheGetDoublev(cache, HL_PROXY_POSITION, pPointMass->m_position);
	global_position=pPointMass->m_position;

	recv(Connection, rec_buffer, sizeof(rec_buffer), NULL);

	sscanf_s(rec_buffer, "%f %f %f", &result_x, &result_y, &result_z);

	
	force[0]=result_x/3;
	force[1]=result_y/3;
	force[2]=result_z/3;
	memset(rec_buffer, 0, sizeof(rec_buffer));
}


/******************************************************************************
 Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
    PointMass *pPointMass = (PointMass *) userdata;
    
    fprintf(stdout, "Custom effect started\n");

    // Initialize the position of the mass to be at the proxy position.
    hlCacheGetDoublev(cache, HL_PROXY_POSITION, pPointMass->m_position);

}


/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
    fprintf(stdout, "Custom effect stopped\n");
}


/******************************************************************************
 Initializes the control parameters used for simulating the point mass.
******************************************************************************/
void initPointMass(PointMass *pPointMass)
{
    pPointMass->m_mass = 0.001; // Kg        

    // Query HDAPI for the max spring stiffness and then tune it down to allow
    // for stable force rendering throughout the workspace.
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &pPointMass->m_kStiffness);
    pPointMass->m_kStiffness *= 0.5;

    // Compute damping constant so that the point mass motion is
    // critically damped.
    pPointMass->m_kDamping = 2 * sqrt(pPointMass->m_mass *
                                      pPointMass->m_kStiffness);
}

int main() 
{
	// Winsock Startup
	WSAData wsaData;
	WORD DllVersion = MAKEWORD(2, 1);

	if (WSAStartup(DllVersion, &wsaData) != 0)
	{
		MessageBoxA(NULL, "Winsock startup failed", "Error", MB_OK | MB_ICONERROR);
		exit(1);
	}

	SOCKADDR_IN Address; // Address to be binded to our Connection socket
	int AddressSize = sizeof(Address); // Need AddressSize for the connect function below
	Address.sin_addr.s_addr = inet_addr("127.0.0.1"); // local IP Address
	Address.sin_port = htons(5555); // Port
	Address.sin_family = AF_INET; // Defines type of socket (IPv4 Socket)

	// Create socket and bind address
	Connection = socket(AF_INET, SOCK_STREAM, NULL);

	if (connect(Connection, (SOCKADDR*)&Address, AddressSize))
	{
		MessageBoxA(NULL, "Failed to Connect", "Error", MB_OK | MB_ICONERROR);
		return 0;
	}

	std::cout << "Connected to server" << std::endl;


	//Haptics initialization

	HHD hHD;
    HHLRC hHLRC;
    HDErrorInfo error;

    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        return -1;
    }
    hdMakeCurrentDevice(hHD);

    hHLRC = hlCreateContext(hHD);
    hlMakeCurrent(hHLRC);
    
    hlDisable(HL_USE_GL_MODELVIEW);

	 HLuint effect = hlGenEffects(1);        

	//send Threading 
	HANDLE send_Thread;
	unsigned send_threadID;

	send_Thread = (HANDLE)_beginthreadex(NULL, 0, &send_position, 0, 0, &send_threadID);
    
	
	// Initialize the point mass.
    PointMass pointMass;
    initPointMass(&pointMass);

    hlBeginFrame();

    hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc) computeForceCB, &pointMass);
    hlCallback(HL_EFFECT_START, (HLcallbackProc) startEffectCB, &pointMass);
    hlCallback(HL_EFFECT_STOP, (HLcallbackProc) stopEffectCB, &pointMass);

    hlStartEffect(HL_EFFECT_CALLBACK, effect);

    hlEndFrame();

	WaitForSingleObject(send_Thread, INFINITE);

    fprintf(stdout, "Press any key to stop the effect\n");
    while (!_kbhit())
    {
        // Check for any errors.
        HLerror error;
        while (HL_ERROR(error = hlGetError()))
        {
            fprintf(stderr, "HL Error: %s\n", error.errorCode);
            
            if (error.errorCode == HL_DEVICE_ERROR)
            {
                hduPrintError(stderr, &error.errorInfo,
                    "Error during haptic rendering\n");
            }
        }                  
    }

    hlBeginFrame();
    hlStopEffect(effect);
    hlEndFrame();
	CloseHandle(send_Thread);


    fprintf(stdout, "Shutting down...\n");
    getchar();

    hlDeleteEffects(effect, 1);

    hlDeleteContext(hHLRC);
    hdDisableDevice(hHD);

	return 0;
}