#include <iostream>
#include <winsock2.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <sstream>
#include <process.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <time.h>
#include <Windows.h>


#pragma comment (lib, "Ws2_32.lib")
using namespace std;
int countround = 0;
float k=0.08;   //Spring constant
hduVector3Dd result_force; //for constantly sending force to the client

//spring force calculation
hduVector3Dd Force_calculation(hduVector3Dd client_pos ) {
	hduVector3Dd force;
	
	force = -k * client_pos;

	return force;
	
}


//recieving position thread
unsigned int _stdcall recieve_position(void* data) {
	char rec_buffer[1024];
	int* client_ptr = (int*)data;
	int client = *client_ptr;

	while (true)
	{
		//recieve position from client
		recv(client, rec_buffer, sizeof(rec_buffer), 0);
		
		float client_x, client_y, client_z = 0.0;

		//read from the buffer
		sscanf_s(rec_buffer, "%f %f %f", &client_x, &client_y, &client_z);

		//force calculation
		hduVector3Dd client_pos;
		client_pos[0] = client_x;
		client_pos[1] = client_y;
		client_pos[2] = client_z;

		result_force = Force_calculation(client_pos);

		memset(rec_buffer, 0, sizeof(rec_buffer));
	}

		return 0;
}

// sending force thread
unsigned int _stdcall send_force(void* data) {
	char send_buffer[1024];
	int* client_ptr = (int*)data;
	int client = *client_ptr;
	
	while (true) {
		int j;
		j = sprintf_s(send_buffer, 1024, "%f\n", result_force[0]);
		j += sprintf_s(send_buffer + j, 1024 - j, "%f\n", result_force[1]);
		j += sprintf_s(send_buffer + j, 1024 - j, "%f\n", result_force[2]);
		send(client, send_buffer, sizeof(send_buffer), NULL);
		memset(send_buffer, 0, sizeof(send_buffer));
		cout << "force send: " << result_force[0] << " " << result_force[1] << " " << result_force[2] << endl;
		//Sleep(1);
	}
	
	return 0;

}

int main()
{
	WSADATA WSAData;
	SOCKET server, client;
	SOCKADDR_IN serverAddr, clientAddr;

	WSAStartup(MAKEWORD(2, 0), &WSAData);
	server = socket(AF_INET, SOCK_STREAM, 0);

	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(5555);

	bind(server, (SOCKADDR *)&serverAddr, sizeof(serverAddr));
	if ((listen(server, 0)) != 0) {
		cout << "Listen failed" << endl;
	}
	else {
	 cout << "Listening for incoming connections..." << endl;
	}

	
	int clientAddrSize = sizeof(clientAddr);
	if ((client = accept(server, (SOCKADDR *)&clientAddr, &clientAddrSize)) != INVALID_SOCKET)
	{
		cout << "Client connected!" << endl;
		
	}


	//Threading
	HANDLE rec_Thread;
	unsigned rec_threadID;

	HANDLE send_Thread;
	unsigned send_threadID;

	//creating thread
	rec_Thread = (HANDLE)_beginthreadex(NULL, 0, &recieve_position, (void*)&client, 0, &rec_threadID);
	send_Thread = (HANDLE)_beginthreadex(NULL, 0, &send_force, (void*)&client, 0, &send_threadID);

	WaitForSingleObject(rec_Thread, INFINITE);
	WaitForSingleObject(send_Thread, INFINITE);

	CloseHandle(rec_Thread);
	CloseHandle(send_Thread);

	
	system("pause");

	return 0;
}