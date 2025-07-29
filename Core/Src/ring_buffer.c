#include "ring_buffer.h"
#include <string.h>

void ring_buffer_push(struct ring_buffer *rb, char *data)
{
    if (ring_buffer_full(rb))
        return;
    rb->buf[rb->head] = *data;
    rb->head++;
    if (rb->head > rb->buf_size)
        rb->head = 0;
}

uint8_t ring_buffer_pop(struct ring_buffer *rb)
{
    if (ring_buffer_empty(rb))
        return 0;

    const uint8_t tmp = rb->buf[rb->tail];
    rb->tail++;
    if (rb->tail > rb->buf_size)
        rb->tail = 0;

    return tmp;
}

void ring_buffer_peek_head(const struct ring_buffer *rb, void *data, uint8_t offset)
{
    int16_t offset_idx = ((int16_t)rb->head - 1) - offset;
    if (offset_idx < 0)
        offset_idx = rb->buf_size + offset_idx;

    memcpy(data, &rb->buf[rb->tail * rb->elem_size], rb->elem_size);
    return;
}

void ring_buffer_peek_tail(const struct ring_buffer *rb, void *data)
{
    memcpy(data, &rb->buf[rb->tail * rb->elem_size], rb->elem_size);
    return;
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
