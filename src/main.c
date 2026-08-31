#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "read.h"
#include "en_dc.h"
#include "read_file.h"
#include "drive.h"

#define NUM_PRODUCERS 1
#define NUM_CONSUMERS 3

/* Shared communication objects */
Message_Queue queue;
Shared_Buffer shared_buffer;
ReadWrite_Lock lock;

pthread_mutex_t message_mutex;
pthread_cond_t message_available;

int producer_finished = 0;

/*
 * Producer:
 * Reads coordinates from the input file, converts them into bytes,
 * encodes them and places one message at a time into the shared buffer.
 */
void *producer(void *arg)
{
    FileArgs *args = (FileArgs *)arg;
    InputFile input;

    if (input_file_open(&input, args->filename) != 0) {
        printf("Failed to open input file: %s\n", args->filename);

        pthread_mutex_lock(&message_mutex);
        producer_finished = 1;
        pthread_cond_broadcast(&message_available);
        pthread_mutex_unlock(&message_mutex);

        return NULL;
    }

    float x_coord;
    float y_coord;

    while (input_file_read(&input, &x_coord, &y_coord)) {

        Message msg = {0};

        /*
         * Store the two float coordinates as raw bytes.
         */
        uint8_t coordinate_data[sizeof(float) * 2];

        memcpy(coordinate_data, &x_coord, sizeof(float));
        memcpy(coordinate_data + sizeof(float),
               &y_coord,
               sizeof(float));

        /*
         * Encode the coordinate data.
         */
        encode_result encoded = frame_encode(
            msg.data,
            max_size,
            coordinate_data,
            sizeof(coordinate_data)
        );

        if (encoded.status != ENCODE_OK) {
            printf("Encoding failed for coordinate %.3f %.3f\n",
                   x_coord, y_coord);
            continue;
        }

        msg.length = encoded.out_len;

        /*
         * Wait until the shared buffer is empty.
         */
        pthread_mutex_lock(&message_mutex);

        while (shared_buffer.length != 0) {
            pthread_cond_wait(&message_available, &message_mutex);
        }

        /*
         * Store exactly one message in the shared buffer.
         */
        shared_buffer.length = msg.length;
        memcpy(shared_buffer.data,
               msg.data,
               msg.length);

        /*
         * Notify consumers.
         */
        pthread_cond_broadcast(&message_available);

        pthread_mutex_unlock(&message_mutex);
    }

    input_file_close(&input);

    /*
     * Tell consumers that the producer has finished.
     */
    pthread_mutex_lock(&message_mutex);

    producer_finished = 1;

    pthread_cond_broadcast(&message_available);

    pthread_mutex_unlock(&message_mutex);

    return NULL;
}


/*
 * Consumer:
 * Retrieves encoded messages from the shared buffer, decodes them
 * and forwards them to the drive queue.
 */
void *consumer(void *arg)
{
    int id = *(int *)arg;

    (void)id;

    while (1) {

        Message msg = {0};

        /*
         * Wait for a message.
         */
        pthread_mutex_lock(&message_mutex);

        while (shared_buffer.length == 0 && !producer_finished) {
            pthread_cond_wait(&message_available, &message_mutex);
        }

        /*
         * If production has finished and there is no message,
         * this consumer can terminate.
         */
        if (shared_buffer.length == 0 && producer_finished) {
            pthread_mutex_unlock(&message_mutex);
            break;
        }

        /*
         * Copy the shared message locally.
         */
        msg.length = shared_buffer.length;

        memcpy(msg.data,
               shared_buffer.data,
               shared_buffer.length);

        /*
         * Clear the shared buffer.
         */
        shared_buffer.length = 0;

        /*
         * Wake the producer so it can place the next message.
         */
        pthread_cond_broadcast(&message_available);

        pthread_mutex_unlock(&message_mutex);

        /*
         * Decode the message.
         */
        uint8_t decoded_data[sizeof(float) * 2];

        decode_result decoded = frame_decode(
            decoded_data,
            sizeof(decoded_data),
            msg.data,
            msg.length
        );

        if (decoded.status != DECODE_OK ||
            decoded.out_len != sizeof(decoded_data)) {

            printf("Consumer %d: decoding failed\n", id);
            continue;
        }

        /*
         * Forward the decoded data to the drive queue.
         */
        Message drive_msg = {0};

        drive_msg.length = decoded.out_len;

        memcpy(drive_msg.data,
               decoded_data,
               decoded.out_len);

        if (message_queue_push(&queue, &drive_msg) != 0) {
            printf("Consumer %d: queue push failed\n", id);
        }
    }

    return NULL;
}


/*
 * Drive thread:
 * Receives coordinate messages from the queue and moves the rover
 * to each target.
 */
void *drive_write(void *arg)
{
    FileArgs *args = (FileArgs *)arg;
    InputFile output;

    if (input_file_open_write(&output, args->result_filename) != 0) {
        printf("Failed to open result file: %s\n",
               args->result_filename);
        return NULL;
    }

    struct rover_state rover;

    rover.position.latitude = 0.0f;
    rover.position.longitude = 0.0f;
    rover.position.altitude = 0.0f;
    rover.heading_rad = 0.0f;

    while (1) {

        Message msg = {0};

        /*
         * Wait for a message from the consumers.
         */
        if (message_queue_pop(&queue, &msg) != 0) {
            break;
        }

        /*
         * A zero-length message is used as the termination signal.
         */
        if (msg.length == 0) {
            break;
        }

        if (msg.length != sizeof(float) * 2) {
            printf("Invalid coordinate message length\n");
            continue;
        }

        float latitude;
        float longitude;

        memcpy(&latitude,
               msg.data,
               sizeof(float));

        memcpy(&longitude,
               msg.data + sizeof(float),
               sizeof(float));

        struct coordinate target;

        target.latitude = latitude;
        target.longitude = longitude;
        target.altitude = rover.position.altitude;

        /*
         * Drive to the target.
         */
        enum drive_status result_status =
            drive_to_target(&rover, &target);

        float dx =
            target.latitude - rover.position.latitude;

        float dy =
            target.longitude - rover.position.longitude;

        float error = hypotf(dx, dy);

        int status =
            (result_status == DRIVE_REACHED_TARGET &&
             error <= 0.10f)
                ? 0
                : 1;

        /*
         * Write the rover result.
         */
        input_file_write(
            &output,
            &rover.position.latitude,
            &rover.position.longitude,
            &error,
            &status
        );
    }

    input_file_close(&output);

    return NULL;
}


int main(void)
{
    pthread_t producers[NUM_PRODUCERS];
    pthread_t consumers[NUM_CONSUMERS];
    pthread_t drive_writers[NUM_PRODUCERS];

    int consumer_id[NUM_CONSUMERS] = {1, 2, 3};

    const char *testcases[] = {
        "input/testcase1.txt",
        "input/testcase2.txt",
        "input/testcase3.txt",
        "input/testcase4.txt"
    };

    const char *result_tc[] = {
        "result/result1.txt",
        "result/result2.txt",
        "result/result3.txt",
        "result/result4.txt"
    };

    /*
     * Initialize synchronization.
     */
    if (rwlock_init(&lock) != 0) {
        printf("Reader writer synchronization failed\n");
        return 1;
    }

    if (message_queue_init(&queue) != 0) {
        printf("Queue initialization failed\n");
        rwlock_destroy(&lock);
        return 1;
    }

    if (pthread_mutex_init(&message_mutex, NULL) != 0) {
        printf("Message mutex initialization failed\n");
        message_destroy(&queue);
        rwlock_destroy(&lock);
        return 1;
    }

    if (pthread_cond_init(&message_available, NULL) != 0) {
        printf("Condition variable initialization failed\n");
        pthread_mutex_destroy(&message_mutex);
        message_destroy(&queue);
        rwlock_destroy(&lock);
        return 1;
    }

    /*
     * Run all four test cases.
     */
    for (int testcase = 0; testcase < 4; testcase++) {

        printf("\n==============================\n");
        printf("Input : %d\n", testcase + 1);
        printf("==============================\n");

        FileArgs file_args = {
            .id = testcase + 1,
            .filename = testcases[testcase],
            .result_filename = result_tc[testcase]
        };

        /*
         * Reset shared state for this test case.
         */
        pthread_mutex_lock(&message_mutex);

        shared_buffer.length = 0;
        producer_finished = 0;

        pthread_mutex_unlock(&message_mutex);

        /*
         * Start producer.
         */
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            if (pthread_create(
                    &producers[i],
                    NULL,
                    producer,
                    &file_args) != 0) {

                printf("Failed to create producer thread\n");
            }
        }

        /*
         * Start consumers.
         */
        for (int i = 0; i < NUM_CONSUMERS; i++) {
            if (pthread_create(
                    &consumers[i],
                    NULL,
                    consumer,
                    &consumer_id[i]) != 0) {

                printf("Failed to create consumer thread %d\n",
                       i + 1);
            }
        }

        /*
         * Start drive writer.
         */
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            if (pthread_create(
                    &drive_writers[i],
                    NULL,
                    drive_write,
                    &file_args) != 0) {

                printf("Failed to create drive thread\n");
            }
        }

        /*
         * Wait for producer.
         */
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_join(producers[i], NULL);
        }

        /*
         * Wait for all consumers.
         */
        for (int i = 0; i < NUM_CONSUMERS; i++) {
            pthread_join(consumers[i], NULL);
        }

        /*
         * Send termination message to the drive thread.
         */
        Message end_message = {0};
        end_message.length = 0;

        message_queue_push(&queue, &end_message);

        /*
         * Wait for drive thread.
         */
        for (int i = 0; i < NUM_PRODUCERS; i++) {
            pthread_join(drive_writers[i], NULL);
        }

        printf("Test case %d completed.\n", testcase + 1);
    }

    /*
     * Cleanup.
     */
    pthread_cond_destroy(&message_available);
    pthread_mutex_destroy(&message_mutex);

    message_destroy(&queue);
    rwlock_destroy(&lock);

    printf("\nAll test cases completed.\n");

    return 0;
}
