#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

static int shared_counter = 0;
static pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t sync_semaphore;

void init_synchronization(void) {
    pthread_mutex_init(&counter_mutex, NULL);
    sem_init(&sync_semaphore, 0, 0);
}

void increment_counter(void) {
    pthread_mutex_lock(&counter_mutex);
    shared_counter++;
    pthread_mutex_unlock(&counter_mutex);
}

int get_counter(void) {
    int val;
    pthread_mutex_lock(&counter_mutex);
    val = shared_counter;
    pthread_mutex_unlock(&counter_mutex);
    return val;
}

void signal_worker(void) {
    sem_post(&sync_semaphore);
}

void wait_for_signal(void) {
    sem_wait(&sync_semaphore);
}

void cleanup_synchronization(void) {
    pthread_mutex_destroy(&counter_mutex);
    sem_destroy(&sync_semaphore);
}
