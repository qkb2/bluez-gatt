#include "ble_client.h"
#include "http_server.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

static void* ble_thread(void* arg) {
    ble_client_start();
    /* BlueZ mainloop runs internally */
    return NULL;
}

int main(void) {
    pthread_t ble_tid;

    if (pthread_create(&ble_tid, NULL, ble_thread, NULL) != 0) {
        perror("pthread_create");
        return EXIT_FAILURE;
    }

    /* HTTP server runs in main thread */
    http_server_run(8080);

    return 0;
}