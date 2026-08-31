#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <semaphore.h>

#include "read.h"
#include "en_dc.h"
#include "read_file.h"
#include "drive.h"

#define NUM_PRODUCERS 1
#define NUM_CONSUMERS 3
#define NUM_TESTCASES 4

#define MESSAGE_END 0xFFFFFFFFu

static Message_Queue input_queue;
static Message_Queue drive_queue;

static void *producer(void *arg)
{
    FileArgs *args = (FileArgs *)arg;
    InputFile input;

    if (input_file_open(&input, args->filename) != 0) {
        return NULL;
    }

    float x;
    float y;

    while (input_file_read(&input, &x, &y)) {

        /*
         * Store the two coordinates as raw float bytes.
         */
        uint8_t source[sizeof(float) * 2];

        memcpy(source, &x, sizeof(float));
        memcpy(source + sizeof(float), &y, sizeof(float));

        Message encoded = {0};

        encode_result enc = frame_encode(
            encoded.data,
            max_size,
            source,
            sizeof(source)
        );

        if (enc.status != ENCODE_OK) {
            printf("Encoding failed\n");
            continue;
        }

        encoded.length = enc.out_len;

        if (message_queue_push(&input_queue, &encoded) != 0) {
            printf("Failed to push message\n");
            break;
        }
    }

    input_file_close(&input);

    /*
     * Send one termination message for each consumer.
     */
    for (int i = 0; i < NUM_CONSUMERS; i++) {
        Message end_message = {0};
        end_message.length = MESSAGE_END;
        message_queue_push(&input_queue, &end_message);
    }

    return NULL;
}


static void *consumer(void *arg)
{
    int id = *(int *)arg;

    while (1) {

        Message encoded = {0};

        if (message_queue_pop(&input_queue, &encoded) != 0) {
            break;
        }

        /*
         * Special message tells this consumer to stop.
         */
        if (encoded.length == MESSAGE_END) {
            break;
        }

        Message decoded = {0};

        decode_result dec = frame_decode(
            decoded.data,
            max_size,
            encoded.data,
            encoded.length
        );

        if (dec.status != DECODE_OK) {
            printf("Consumer %d: decoding failed\n", id);
            continue;
        }

        decoded.length = dec.out_len;

        if (message_queue_push(&drive_queue, &decoded) != 0) {
            printf("Consumer %d: failed to forward message\n", id);
            break;
        }
    }

    return NULL;
}


static void *drive_write(void *arg)
{
    FileArgs *args = (FileArgs *)arg;
    InputFile output;

    if (input_file_open_write(&output, args->result_filename) != 0) {
        printf("Failed to open result file %s\n",
               args->result_filename);
        return NULL;
    }

    /*
     * Process messages until main sends the termination message.
     */
    while (1) {

        Message msg = {0};

        if (message_queue_pop(&drive_queue, &msg) != 0) {
            break;
        }

        if (msg.length == MESSAGE_END) {
            break;
        }

        if (msg.length != sizeof(float) * 2) {
            printf("Invalid coordinate message\n");
            continue;
        }

        float x;
        float y;

        memcpy(&x, msg.data, sizeof(float));
        memcpy(&y, msg.data + sizeof(float), sizeof(float));

        struct rover_state rover;

        rover.position.latitude = 0.0f;
        rover.position.longitude = 0.0f;
        rover.position.altitude = 0.0f;
        rover.heading_rad = 0.0f;

        struct coordinate target;

        target.latitude = x;
        target.longitude = y;
        target.altitude = 0.0f;

        enum drive_status status =
            drive_to_target(&rover, &target);

        float dx = target.latitude - rover.position.latitude;
        float dy = target.longitude - rover.position.longitude;
        float error = hypotf(dx, dy);

        int result_status =
            (status == DRIVE_REACHED_TARGET) ? 0 : 1;

        input_file_write(
            &output,
            &rover.position.latitude,
            &rover.position.longitude,
            &error,
            &result_status
        );
    }

    input_file_close(&output);

    return NULL;
}


int main(void)
{
    const char *testcases[NUM_TESTCASES] = {
        "input/testcase1.txt",
        "input/testcase2.txt",
        "input/testcase3.txt",
        "input/testcase4.txt"
    };

    const char *result_files[NUM_TESTCASES] = {
        "result/result1.txt",
        "result/result2.txt",
        "result/result3.txt",
        "result/result4.txt"
    };

    if (message_queue_init(&input_queue) != 0) {
        printf("Input queue initialization failed\n");
        return 1;
    }

    if (message_queue_init(&drive_queue) != 0) {
        printf("Drive queue initialization failed\n");
        message_destroy(&input_queue);
        return 1;
    }

    for (int test = 0; test < NUM_TESTCASES; test++) {

        printf("\n==============================\n");
        printf("Running testcase %d\n", test + 1);
        printf("==============================\n");

        FileArgs args;

        args.id = test + 1;
        args.filename = testcases[test];
        args.result_filename = result_files[test];

        pthread_t producer_thread;
        pthread_t consumer_threads[NUM_CONSUMERS];
        pthread_t drive_thread;

        int consumer_ids[NUM_CONSUMERS] = {1, 2, 3};

        /*
         * Start the drive thread first so it can wait for messages.
         */
        pthread_create(
            &drive_thread,
            NULL,
            drive_write,
            &args
        );

        /*
         * Start consumers.
         */
        for (int i = 0; i < NUM_CONSUMERS; i++) {
            pthread_create(
                &consumer_threads[i],
                NULL,
                consumer,
                &consumer_ids[i]
            );
        }

        /*
         * Start producer.
         */
        pthread_create(
            &producer_thread,
            NULL,
            producer,
            &args
        );

        pthread_join(producer_thread, NULL);

        for (int i = 0; i < NUM_CONSUMERS; i++) {
            pthread_join(consumer_threads[i], NULL);
        }

        /*
         * Tell the drive thread that all consumers are finished.
         */
        Message end_message = {0};
        end_message.length = MESSAGE_END;

        message_queue_push(
            &drive_queue,
            &end_message
        );

        pthread_join(drive_thread, NULL);

        printf("Testcase %d completed\n", test + 1);
    }

    message_destroy(&input_queue);
    message_destroy(&drive_queue);

    printf("\nAll testcases completed.\n");

    return 0;
}
