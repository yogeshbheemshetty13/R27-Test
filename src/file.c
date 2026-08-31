#include <stdio.h>
#include <stddef.h>

#include "read_file.h"

int input_file_open(InputFile *input, const char *filename)
{
    if (input == NULL || filename == NULL) {
        return -1;
    }

    input->file = fopen(filename, "r");

    if (input->file == NULL) {
        perror("Failed to open input file");
        return -1;
    }

    return 0;
}

int input_file_open_write(InputFile *file, const char *filename)
{
    if (file == NULL || filename == NULL) {
        return -1;
    }

    file->file = fopen(filename, "w");

    if (file->file == NULL) {
        perror("Failed to open output file");
        return -1;
    }

    return 0;
}

int input_file_write(InputFile *file,
                     float *x,
                     float *y,
                     float *error,
                     int *status)
{
    if (file == NULL ||
        file->file == NULL ||
        x == NULL ||
        y == NULL ||
        error == NULL ||
        status == NULL) {
        return -1;
    }

    if (fprintf(file->file,
                "%.2f %.2f %.2f %d\n",
                *x,
                *y,
                *error,
                *status) < 0) {
        return -1;
    }

    return 0;
}

int input_file_read(InputFile *input, float *x, float *y)
{
    if (input == NULL ||
        input->file == NULL ||
        x == NULL ||
        y == NULL) {
        return 0;
    }

    return fscanf(input->file, "%f %f", x, y) == 2;
}

void input_file_close(InputFile *input)
{
    if (input != NULL && input->file != NULL) {
        fclose(input->file);
        input->file = NULL;
    }
}
