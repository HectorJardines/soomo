#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

struct ring_buffer
{
    uint8_t *buf;
    uint8_t buf_size;
    uint8_t elem_size;
    uint8_t head;
    uint8_t tail;
};

void ring_buffer_push(struct ring_buffer *buf, char *data);
uint8_t ring_buffer_pop(struct ring_buffer *buf);
void ring_buffer_peek_tail(const struct ring_buffer *rb, void *data);
void ring_buffer_peek_head(const struct ring_buffer *rb, void *data, uint8_t offset);
bool ring_buffer_full(struct ring_buffer *rb);
bool ring_buffer_empty(struct ring_buffer *rb);

#endif