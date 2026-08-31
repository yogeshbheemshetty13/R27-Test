#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include "read.h"

#define QUEUE_SIZE 50

int message_queue_init(Message_Queue *queue)
{
    if (queue == NULL) {
        return -1;
    }

    queue->head = 0;
    queue->tail = 0;
    queue->current = 0;

    if (pthread_mutex_init(&queue->mutex, NULL) != 0) {
        return -1;
    }

    if (sem_init(&queue->empty, 0, QUEUE_SIZE) != 0) {
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }

    if (sem_init(&queue->full, 0, 0) != 0) {
        sem_destroy(&queue->empty);
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }

    return 0;
}

void message_destroy(Message_Queue *queue)
{
    if (queue == NULL) {
        return;
    }

    pthread_mutex_destroy(&queue->mutex);
    sem_destroy(&queue->full);
    sem_destroy(&queue->empty);
}

int message_queue_push(Message_Queue *queue, const Message *msg)
{
    if (queue == NULL || msg == NULL) {
        return -1;
    }

    /*
     * Wait until there is an empty space in the queue.
     */
    if (sem_wait(&queue->empty) != 0) {
        return -1;
    }

    /*
     * Only one thread can modify the queue at a time.
     */
    if (pthread_mutex_lock(&queue->mutex) != 0) {
        sem_post(&queue->empty);
        return -1;
    }

    /*
     * Copy the message into the queue.
     */
    queue->buffer[queue->tail] = *msg;

    /*
     * Move tail to the next position.
     */
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;

    queue->current++;

    pthread_mutex_unlock(&queue->mutex);

    /*
     * Tell waiting consumers that a message is available.
     */
    sem_post(&queue->full);

    return 0;
}

int message_queue_pop(Message_Queue *queue, Message *msg)
{
    if (queue == NULL || msg == NULL) {
        return -1;
    }

    /*
     * Wait until there is at least one message.
     */
    if (sem_wait(&queue->full) != 0) {
        return -1;
    }

    /*
     * Only one thread can modify the queue at a time.
     */
    if (pthread_mutex_lock(&queue->mutex) != 0) {
        sem_post(&queue->full);
        return -1;
    }

    /*
     * Copy the message out of the queue.
     */
    *msg = queue->buffer[queue->head];

    /*
     * Move head to the next position.
     */
    queue->head = (queue->head + 1) % QUEUE_SIZE;

    queue->current--;

    pthread_mutex_unlock(&queue->mutex);

    /*
     * Tell the producer that an empty space is available.
     */
    sem_post(&queue->empty);

    return 0;
}
