#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define MAX_MSG_SIZE 256
void error_handling(char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}
int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s <server_ip> <server_port> <nickname>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    int client_socket, port;
    struct sockaddr_in server_addr;
    char buffer[MAX_MSG_SIZE];

    // 서버 IP 및 포트 가져오기
    

    // 소켓 생성
    if ((client_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        error_handling("Socket creation failed");
    }

    // 서버 주소 설정
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(argv[1]);
    server_addr.sin_port = htons(atoi(argv[2]));

    // 서버에 연결
    if (connect(client_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        error_handling("Connection failed");
    }

    // 닉네임 전송
    send(client_socket, argv[3], strlen(argv[3]), 0);
    printf("Type 'Q' or 'q' to exit the chat\n");

    fd_set read_fds;
    
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        FD_SET(client_socket, &read_fds);

        if (select(client_socket + 1, &read_fds, NULL, NULL, NULL) == -1) {
            error_handling("Select failed");
        }

        


        // 서버로부터 닉네임메시지 수신 및 출력
        if (FD_ISSET(client_socket, &read_fds)) {
            memset(buffer, 0, sizeof(buffer));
            if (recv(client_socket, buffer, MAX_MSG_SIZE, 0) > 0) {
                printf("[%s]\n", buffer);
            } else {
                
                perror("Receive failed");
                break;
            }
        }
        
        // 사용자 입력을 서버로 전송
        if (FD_ISSET(STDIN_FILENO, &read_fds)) {
            memset(buffer, 0, sizeof(buffer));
            
            fgets(buffer, sizeof(buffer), stdin);
            if(!strcmp(buffer, "q\n") || !strcmp(buffer, "Q\n"))
                break; 

            if (send(client_socket, buffer, strlen(buffer), 0) == -1) {
                perror("Send failed");
                break;
            }
        }
    }
    
    FD_CLR(STDIN_FILENO, &read_fds);
    FD_CLR(client_socket, &read_fds);
    close(client_socket);

    return 0;
}