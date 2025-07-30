#include "ring_buffer.h"
#include <string.h>

void ring_buffer_push(struct ring_buffer *rb, void *data)
{
    // {} head = 0, tail = 0
    // {1} head = 1, tail = 0
    // {1, 2} head = 2, tail = 0
    // {1, 2, 3} head = 3, tail = 0
    // {1, 2, 3, 4} head = 4, tail = 0
    if (ring_buffer_full(rb))
        ring_buffer_pop(rb, NULL);
    memcpy(&rb->buf[rb->head * rb->elem_size], data, rb->elem_size);

    rb->head++;
    if (rb->head >= rb->buf_size)
        rb->head = 0;
}

void ring_buffer_pop(struct ring_buffer *rb, void *data)
{
    if (data)
        memcpy(data, &rb->buf[rb->head * rb->elem_size], rb->elem_size);

    rb->tail++;
    if (rb->tail >= rb->buf_size)
        rb->tail = 0;
}

void ring_buffer_peek_head(const struct ring_buffer *rb, void *data, uint8_t offset)
{
    int16_t offset_idx = ((int16_t)rb->head - 1) - offset;
    if (offset_idx < 0)
        offset_idx = rb->buf_size + offset_idx;

    memcpy(data, &rb->buf[rb->tail * rb->elem_size], rb->elem_size);
}

void ring_buffer_peek_tail(const struct ring_buffer *rb, void *data)
{
    memcpy(data, &rb->buf[rb->tail * rb->elem_size], rb->elem_size);
}

bool ring_buffer_full(struct ring_buffer *rb)
{
    uint8_t idx_after_head = rb->head + 1;
    if (idx_after_head + 1 > rb->buf_size)
        idx_after_head = 0;
    return idx_after_head == rb->tail;
}

bool ring_buffer_empty(struct ring_buffer *rb)
{
    return rb->tail == rb->head;
}
