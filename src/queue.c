int message_queue_push(Message_Queue *queue, const Message *msg)
{
    if (queue == NULL || msg == NULL) {
        return -1;
    }

    /* Wait until there is an empty slot */
    if (sem_wait(&queue->empty) != 0) {
        return -1;
    }

    /* Protect the queue while modifying it */
    if (pthread_mutex_lock(&queue->mutex) != 0) {
        sem_post(&queue->empty);
        return -1;
    }

    /* Add message at the tail */
    queue->buffer[queue->tail] = *msg;

    /* Move tail to next position */
    queue->tail = (queue->tail + 1) % 50;

    /* Unlock queue */
    pthread_mutex_unlock(&queue->mutex);

    /* Tell consumers that a message is available */
    sem_post(&queue->full);

    return 0;
}
