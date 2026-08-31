void reader_enter(ReadWrite_Lock *lock)
{
    pthread_mutex_lock(&lock->writer_count);

    pthread_mutex_lock(&lock->reader_count);

    lock->reader++;

    /*
     * The first reader locks the shared resource.
     * Other readers can enter while the first reader
     * keeps the resource locked.
     */
    if (lock->reader == 1) {
        sem_wait(&lock->resource);
    }

    pthread_mutex_unlock(&lock->reader_count);

    pthread_mutex_unlock(&lock->writer_count);
}
