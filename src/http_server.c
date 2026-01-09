#include "http_server.h"
#include "ble_client.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define RECV_BUF 1024
#define RESP_BUF 1024

static void send_response(int client_fd) {
    char body[512];
    char response[RESP_BUF];

    float t = 0.0f, p = 0.0f, h = 0.0f;
    bool has_t = ble_get_temperature(&t);
    bool has_p = ble_get_pressure(&p);
    bool has_h = ble_get_humidity(&h);

    snprintf(body, sizeof(body),
             "<!DOCTYPE html>"
             "<html>"
                "<head>"
                    "<title>Sensor Values</title>"
                    "<meta http-equiv=\"refresh\" content=\"5\">"
                "</head>"
                "<body>"
                    "<h1>Sensor Values</h1>"
                    "<p>Temperature: %s</p>"
                    "<p>Pressure: %s</p>"
                    "<p>Humidity: %s</p>"
                "</body>"
             "</html>",
             has_t ? ({
                 static char b[32];
                 snprintf(b, 32, "%.2f °C", t);
                 b;
             })
                   : "N/A",
             has_p ? ({
                 static char b[32];
                 snprintf(b, 32, "%.1f hPa", p);
                 b;
             })
                   : "N/A",
             has_h ? ({
                 static char b[32];
                 snprintf(b, 32, "%.2f %%RH", h);
                 b;
             })
                   : "N/A");

    snprintf(response, sizeof(response),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "Content-Length: %zu\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             strlen(body), body);

    send(client_fd, response, strlen(response), 0);
}

void http_server_run(uint16_t port) {
    int server_fd;
    struct sockaddr_in addr;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen");
        close(server_fd);
        return;
    }

    printf("HTTP server listening on port %u\n", port);

    for (;;) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0)
            continue;

        char buf[RECV_BUF];
        recv(client_fd, buf, sizeof(buf), 0); /* ignore request */

        send_response(client_fd);

        close(client_fd);
    }
}
