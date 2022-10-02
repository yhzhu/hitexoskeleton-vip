#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>

// Need to link with Ws2_32.lib
#pragma comment (lib, "ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "20001"
int* exitrequest = NULL;

WSADATA wsaData;
int iResult;

SOCKET ListenSocket = INVALID_SOCKET;
//SOCKET ClientSocket = INVALID_SOCKET;

struct addrinfo *result = NULL;
struct addrinfo hints;

int iSendResult;
char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;
std::thread recvthread;
std::thread listenthread;

DWORD WINAPI recvThread(void *vargp)
{
    int err = 0;
    printf("Thread got evoked\n");
    SOCKET *pClientSocket = (SOCKET *)vargp;
    SOCKET ClientSocket   = *pClientSocket;  // make a copy of the heap-allocated SOCKET object into a local variable
    free(pClientSocket);                     // then free the heap-allocated SOCKET to avoid a memory leak

    printf("In thread, ClientSocket: %d\n", ClientSocket);
    // // Receive until the peer shuts down the connection
    if(ClientSocket)
    {
        do {
            iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
            if (iResult > 0) {
                printf("Bytes received: %d\n", iResult);

            // Echo the buffer back to the sender
                iSendResult = send( ClientSocket, recvbuf, iResult, 0 );
                if (iSendResult == SOCKET_ERROR) {
                    printf("send failed with error: %d\n", WSAGetLastError());
                    closesocket(ClientSocket);
                    WSACleanup();
                    err = 1;
                    break;
                }
                printf("Bytes sent: %d\n", iSendResult);
            }
            else if (iResult == 0)
            {
                printf("Connection closing...\n");
                closesocket(ClientSocket);
                err = 0;
                break;
            }
            else  {
                printf("recv failed with error: %d\n", WSAGetLastError());
                closesocket(ClientSocket);
                WSACleanup();
                err = 2;
                break;
            }

        } while (iResult > 0);
    }
    printf("recvThread finished err=%d\n",err);
    closesocket(ClientSocket);  // don't forget to close the socket when we're done
    return 0;

}

void listenThread()
{
    int err = 0;
    struct sockaddr_in client_addr;
    char cli_ip[INET_ADDRSTRLEN] = "";
    socklen_t cliaddr_len = sizeof(client_addr);

    while(!(*exitrequest))
    {
        SOCKET * pClientSocket = (SOCKET *) (malloc(sizeof(SOCKET)));  // allocate a SOCKET on the heap
        if (pClientSocket == NULL) {printf("malloc() failed!?\n"); break;}
        // Accept a client socket
        *pClientSocket = accept(ListenSocket, (struct sockaddr*)&client_addr, &cliaddr_len);
        if (*pClientSocket == INVALID_SOCKET) 
        {
            free(pClientSocket);  // avoid memory leak!
            printf("accept failed: %d\n", WSAGetLastError());
            closesocket(ListenSocket);
            WSACleanup();
            return;
        }
        else
        {
            printf("ClientSocket: %d\n", *pClientSocket);
            inet_ntop(AF_INET,&client_addr.sin_addr,cli_ip,INET_ADDRSTRLEN);
            HANDLE thread = CreateThread(NULL, 0, recvThread, pClientSocket, 0, NULL);
            if (thread == NULL)
            {
                closesocket(*pClientSocket);  // avoid socket leak!
                free(pClientSocket);  // avoid memory leak!
                printf("CreateThread failed!?\n");
            }
            printf("client ip=%s,port=%d\n",cli_ip,ntohs(client_addr.sin_port));
        }

        //sleep for 1 ms or yield, to let main thread run
        //yield results in busy wait - which has better timing but kills battery life
        // if( settings.run && settings.busywait )
        //     std::this_thread::yield();
        // else
        printf("listenThread\n");
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    printf("listenThread finished err=%d\n",err);
}

int initTCPServer()
{
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }
    else
        printf("getaddrinfo OK\n");

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }
    else
        printf("create ListenSocket OK\n");

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }   
    else
        printf("bind ListenSocket OK\n");

    // freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
    else
        printf("listen OK\n");

    listenthread = std::thread(listenThread);
    //recvthread = std::thread(recvThread);
    //listenthread.join();
    //recvthread.join();

    return 0;
}

int closeTCPServer()
{
    freeaddrinfo(result);
    // No longer need server socket
    closesocket(ListenSocket);
    // shutdown the connection since we're done
    // iResult = shutdown(ClientSocket, SD_SEND);
    // if (iResult == SOCKET_ERROR) {
    //     printf("shutdown failed with error: %d\n", WSAGetLastError());
    //     closesocket(ClientSocket);
    //     WSACleanup();
    //     return 1;
    // }
    WSACleanup();
    return 0;
}
