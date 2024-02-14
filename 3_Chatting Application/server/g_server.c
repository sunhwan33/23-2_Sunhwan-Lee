#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>

#define MAX_CLIENTS 10
#define MAX_MSG_SIZE 256

typedef struct {
    int socket;
    char nickname[MAX_MSG_SIZE];
} Client;

void handle_client_join(Client *clients, int client_count, int new_socket, const char *nickname) {
    clients[client_count].socket = new_socket;
    strcpy(clients[client_count].nickname, nickname);

    // 알림 메시지 생성
    char join_msg[MAX_MSG_SIZE];
    sprintf(join_msg, "User %s joined the chat.", nickname); 

    // 다른 클라이언트에게 새로운 참여자 알리기
    int i;
    for (i = 0; i < client_count; ++i) {
        send(clients[i].socket, join_msg, strlen(join_msg), 0);
    }
}

void handle_client_leave(Client *clients, int client_count, int leaving_socket) {
    char leave_msg[MAX_MSG_SIZE];
    char leaving_nickname[MAX_MSG_SIZE];
    int leave_index ; 
    // 떠난 사용자의 닉네임 찾기
    int i;
    for (i = 0; i < client_count; ++i) {
        if (clients[i].socket == leaving_socket) {
            leave_index = i;
            strcpy(leaving_nickname, clients[i].nickname);
            break;
        }
    }

    // 알림 메시지 생성
    sprintf(leave_msg, "User %s left the chat.", leaving_nickname);

    // 다른 클라이언트에게 떠난 사용자 알리기
    
    for (i = 0; i < client_count; ++i) {
        send(clients[i].socket, leave_msg, strlen(leave_msg), 0);
    }
    int j;
    for(j = leave_index +1; j<client_count; j++){
        clients[j-1] = clients[j];
    }
    clients[j].socket = -1;
    strcpy(clients[j].nickname, "\0");
    //clients[j].nickname = "\0";
}

void handle_client_message(Client *clients, int client_count, int sender_socket, const char *message) {
    char formatted_msg[MAX_MSG_SIZE];
    char sender_nickname[MAX_MSG_SIZE];
    printf("message : %s\n", message);
    // 보낸 사용자의 닉네임 찾기
    int i;
    for (i = 0; i < client_count; ++i) {
        if (clients[i].socket == sender_socket) {
            strcpy(sender_nickname, clients[i].nickname);
            break;
        }
    }

    // 메시지 포맷팅
    sprintf(formatted_msg, "%s: %s", sender_nickname, message);
    
    // 다른 클라이언트에게 메시지 전달
    for (i = 0; i < client_count; ++i) {
        if (clients[i].socket != sender_socket) {
            send(clients[i].socket, formatted_msg, strlen(formatted_msg)-1, 0);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <port>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    int server_socket, new_socket, port;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[MAX_MSG_SIZE];

    // 포트 번호 가져오기
    port = atoi(argv[1]);

    // 소켓 생성
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 서버 주소 설정
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    // 소켓을 주소와 바인딩
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Socket binding failed");
        exit(EXIT_FAILURE);
    }

    // 연결 대기
    if (listen(server_socket, MAX_CLIENTS) == -1) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    printf("Chat server is listening on port %d...\n", port);

    Client clients[MAX_CLIENTS];
    int client_count = 0;

    fd_set read_fds, temp_fds;

    FD_ZERO(&read_fds);
    FD_SET(server_socket, &read_fds);

    while (1) {
        temp_fds = read_fds;

        // I/O Multiplexing을 통한 소켓 이벤트 감지
        if (select(FD_SETSIZE, &temp_fds, NULL, NULL, NULL) == -1) {
            perror("Select failed");
            exit(EXIT_FAILURE);
        }
        int i;
        for (i = 0; i < FD_SETSIZE; ++i) {
            if (FD_ISSET(i, &temp_fds)) {
                if (i == server_socket) {
                    // 새로운 클라이언트 연결 허용
                    if ((new_socket = accept(server_socket, (struct sockaddr *)&client_addr, &client_len)) == -1) {
                        perror("Accept failed");
                        continue;
                    }

                    // 닉네임 수신
                    memset(buffer, 0, sizeof(buffer));
                    recv(new_socket, buffer, MAX_MSG_SIZE, 0);
                    
                    buffer[strlen(buffer) ] = '\0'; // 개행 문자 제거
                    
                    // 새로운 클라이언트 처리
                    handle_client_join(clients, client_count, new_socket, buffer);
                    client_count++;
                    char sending_msg[MAX_MSG_SIZE+20];
                    // 현재 참여 중인 사용자들에게 닉네임 전송
                    int j;
                    for (j = 0; j < client_count ; ++j) { // client_count -1이 아니라 client_count를 사용함으로써, 새로 참여한 사람에게 그 사람의 닉네임까지도 전달
                        memset(sending_msg, 0, sizeof(sending_msg));
                        sprintf(sending_msg, "client %d : %s",j+1, clients[j].nickname );
                        send(new_socket, sending_msg, strlen(sending_msg), 0);
                        usleep(10000); // 작은 딜레이를 통해 전송 오류 방지
                    }

                    // 새로운 클라이언트를 모니터링 대상에 추가
                    FD_SET(new_socket, &read_fds);
                } else {
                    // 기존 클라이언트의 메시지 처리 또는 연결 끊김 처리
                    memset(buffer, 0, sizeof(buffer));
                    int recv_result = recv(i, buffer, MAX_MSG_SIZE,0);
                    if (recv_result <= 0) {
                        // 클라이언트 연결이 끊겼을 때
                        handle_client_leave(clients, client_count, i);
                        client_count--;

                        close(i);
                        FD_CLR(i, &read_fds);
                    } else {
                        // 메시지가 도착했을 때
                        buffer[recv_result] ='\0';
                        handle_client_message(clients, client_count, i, buffer);
                    }
                }
            }
        }
    }

    close(server_socket);

    return 0;
}
