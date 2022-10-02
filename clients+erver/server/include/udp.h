
#include <stdio.h>
#include<string.h>
#include<errno.h>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32")
#else
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<netdb.h>
#include<arpa/inet.h>
#endif
#include <pthread.h>
#include <thread>
#include <vector>

// #define SERVER_PORT 6555
#define BUFF_LEN 1024
#define DEFAULT_LOCAL_IP    "0.0.0.0"
#define ANY_IP              "0.0.0.0"
#define DEFAULT_LOCAL_PORT  20000
#define DEFAULT_REMOTE_IP   "127.0.0.1"
#define DEFAULT_REMOTE_PORT 20001
int server_fd;
pthread_t serverThread;
char buf[BUFF_LEN];
struct sockaddr_in local_sa;
struct sockaddr_in remote_sa;
socklen_t len;
bool commtoClient = false;
std::vector<struct sockaddr_in*> clients;

typedef void (*ON_MSG)(char* msg);
ON_MSG funccallback;


void * get_in_addr(struct sockaddr * sa)
{
	if (sa->sa_family == AF_INET)
	{
		return &(((struct sockaddr_in *)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}

void ip2addr(const char ip[], unsigned short port, struct sockaddr_in& sa)
{
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = inet_addr(ip);
    sa.sin_port = htons(port);  
}

#ifdef _WIN32

///=================================================================================================
/// <summary>	Determines if winsock is initialized. </summary>
///
/// <remarks>	mtvee, 2017-03-30. </remarks>
///
/// <returns>	True if it is already done. </returns>
///=================================================================================================

bool WinsockInitialized()
{
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s == INVALID_SOCKET && WSAGetLastError() == WSANOTINITIALISED) {
		return false;
	}

	closesocket(s);
	return true;
}
#endif

int udp_send(const char* buf, int len)
{
    sendto(server_fd, buf, len, 0, (struct sockaddr*)&remote_sa, sizeof(remote_sa));
    //printf("msg has been sent!\n");
    return 0;
}

int udp_sendto(const char* ip, unsigned short port, const char* buf, int len)
{
    struct sockaddr_in sa;
    ip2addr(ip,port,sa);
    sendto(server_fd, buf, len, 0, (struct sockaddr*)&sa, sizeof(sa));
    //printf("msg has been sent!\n");
    return 0;
}

void* recvProc(void *arg)
{

    int iRet;
    
    while(1)
    {
        memset(buf, 0, BUFF_LEN);
        len = sizeof(remote_sa);
        iRet = recvfrom(server_fd, buf, sizeof(buf), 0, (struct sockaddr*)&remote_sa, &len);  
      
        if(iRet == -1)
        {
            printf("recieve data fail!\n");
            return NULL;
        }
        int cmp = strncmp(buf,"CMD1",4);
        funccallback(buf);

        if(cmp == 0)
        {
            commtoClient = true;
            clients.push_back(&remote_sa);
            printf("Msg received! client[%s:%d]:%d bytes\n",inet_ntoa(remote_sa.sin_addr),ntohs(remote_sa.sin_port),iRet); 
        }
        
        // if(strncmp(buf,"CMD2",4)==0)
        // {
        //     std::vector<sockaddr_in>::iterator position = std::find(clients.begin(), clients.end(), &clent_addr);
        //     if (position != clients.end()) // == myVector.end() means the element was not found
        //         clients.erase(position);
        // }
        
    }
    return NULL;
}

/*
    server:
            socket-->bind-->recvfrom-->sendto-->close
*/

int udp_init(const char remote_ip[] = DEFAULT_REMOTE_IP, unsigned short remote_port = DEFAULT_REMOTE_PORT, unsigned short local_port = DEFAULT_LOCAL_PORT, ON_MSG callback = NULL)
{
    int ret;

    ip2addr(remote_ip, remote_port, remote_sa);
    ip2addr(DEFAULT_LOCAL_IP, local_port, local_sa);
    funccallback = callback;

#ifdef _WIN32
	//----------------------
	// Initialize Winsock.
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR) {
		wprintf(L"WSAStartup failed with error: %ld\n", iResult);
		return 1;
	}
#endif

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        return -1;
    }

    ret = bind(server_fd, (struct sockaddr*)&local_sa, sizeof(local_sa));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        return -1;
    }
    printf("======waiting for client's request======\n");

    pthread_create(&serverThread, NULL, &recvProc, NULL);
    return 0;
}

int udp_close()
{
#ifdef _WIN32
    closesocket(server_fd);
    WSACleanup();
#else
	close(server_fd);
#endif
    return 0;
}
