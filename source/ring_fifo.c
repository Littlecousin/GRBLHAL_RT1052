#include "ring_fifo.h"

#include <assert.h>

#define MIN(a, b)       ((a) > (b) ? (b) : (a))
#define MAX_FIFO_DEPTH  (0xffffffff >> 1)

#undef RING_PARAMETER_VERIFY
#define RING_PARAMETER_VERIFY(expr) do {    \
    if(expr)                                \
    {                                       \
        return RF_INVALID_PARAMETER;        \
    }                                       \
}while(0)


static inline uint32_t is_pow_of_2(uint32_t n)
{
    return (0 != n) && (0 == (n & (n - 1)));
}

static inline uint32_t pow2gt(uint32_t x)
{
    --x;

    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;

    return x + 1;
}

uint32_t ring_fifo_init(ring_fifo_t **__ring, uint8_t *pBuf, uint32_t size, enum ring_fifo_type type)
{
    RING_PARAMETER_VERIFY((type != RF_TYPE_FRAME) && (type != RF_TYPE_STREAM));
    ring_fifo_t *_ring = NULL;

    if(NULL != pBuf)
    {
        assert(0 != is_pow_of_2(size));
    }

    _ring = malloc(sizeof(ring_fifo_t));
    if(NULL == _ring)
    {
        return RF_MALLOC_FAILED;
    }

    if(size > MAX_FIFO_DEPTH)
    {
        size = MAX_FIFO_DEPTH;
    }
    _ring->head = 0;
    _ring->tail = 0;
    _ring->size = pow2gt(size);
    if(NULL != pBuf)
    {
        _ring->dataBuf = pBuf;
        _ring->isDynamic = 0;
    }
    else
    {
        _ring->dataBuf = malloc(_ring->size * sizeof(*_ring->dataBuf));
        if(NULL == _ring->dataBuf)
        {
            free(_ring);

            return RF_MALLOC_FAILED;
        }
        _ring->isDynamic = 1;
    }
    memset(_ring->dataBuf, 0, _ring->size * sizeof(*_ring->dataBuf));

    _ring->type = type;

    *__ring = _ring;

    return RF_SUCCESS;
}

uint32_t ring_fifo_write(ring_fifo_t *_ring, const uint8_t *pBuf, uint16_t len)
{
    RING_PARAMETER_VERIFY(NULL == _ring);
    RING_PARAMETER_VERIFY(NULL == pBuf);
    RING_PARAMETER_VERIFY(0 == len);
    uint32_t frame_len = 0;
    uint32_t frame_off = 0;
    uint32_t remain_capacity = 0;

    remain_capacity = _ring->size - (_ring->tail - _ring->head);
    switch(_ring->type)
    {
        case RF_TYPE_FRAME:
        /*如果不能存下此帧，则舍弃*/
        if(len + sizeof(len) > remain_capacity) { return RF_FULL; }
        frame_len = len;
        /*
        *写入帧长
        */
        *(uint16_t *)(&_ring->dataBuf[_ring->tail & (_ring->size - 1)]) = frame_len;
        frame_off += sizeof(len);
        break;
        case RF_TYPE_STREAM:
        /*缓冲区满*/
        if(1 > remain_capacity) { return RF_FULL; }
        /*如果不能全部存下，则丢掉多余数据*/
        frame_len = len > remain_capacity ? remain_capacity : len;
        break;
    }

    /*计算写入位置*/
    uint32_t off = (_ring->tail + frame_off) & (_ring->size - 1);
    /*
    *写入数据
    *如果剩余空间未跨首尾相接的地方，则一次写入，否则分两次写入
    */
    uint32_t min = MIN(frame_len, _ring->size - off);
    memcpy(&(_ring->dataBuf[off]), pBuf, min);
    memcpy(&(_ring->dataBuf[0]), pBuf + min, frame_len - min);
    _ring->tail += frame_len + frame_off;

    return RF_SUCCESS;
}

uint32_t ring_fifo_read(ring_fifo_t *_ring, uint8_t *pBuf, uint16_t len, uint16_t *retLen)
{
    RING_PARAMETER_VERIFY(NULL == _ring);
    RING_PARAMETER_VERIFY(NULL == pBuf);
    RING_PARAMETER_VERIFY(0 == len);
    RING_PARAMETER_VERIFY(NULL == retLen);
    uint32_t frame_len = 0;
    uint32_t frame_off = 0;
    uint32_t used_capacity = 0;

    /*无数据可读*/
    if(_ring->tail == _ring->head) { return RF_EMPTY; }

    used_capacity = _ring->tail - _ring->head;
    switch(_ring->type)
    {
        case RF_TYPE_FRAME:
        /*
        *读取帧长
        */
        frame_len = *(uint16_t *)(&_ring->dataBuf[_ring->head & (_ring->size - 1)]);
        /*内存越界*/
        if(frame_len > len) { return RF_READ_BUFFER_TOO_SMALL; }
        frame_off += sizeof(len);
        break;
        case RF_TYPE_STREAM:
        /*如果可读数据长度大于要读取的长度，则读取指定长度，否则读取可读数据长度*/
        frame_len = used_capacity > len ? len : used_capacity;
        break;
    }

    /*计算读取位置*/
    uint32_t off = (_ring->head + frame_off) & (_ring->size - 1);
    /*
    *读取数据
    *如果剩余空间未跨首尾相接的地方，则一次读出，否则分两次读出
    */
    uint32_t min = MIN(frame_len, _ring->size - off);
    memcpy(pBuf, &(_ring->dataBuf[off]), min);
    memcpy(pBuf + min, &(_ring->dataBuf[0]), frame_len - min);
    _ring->head += frame_len + frame_off;

    *retLen = frame_len;

    return RF_SUCCESS;
}

void ring_fifo_destroy(ring_fifo_t *_ring)
{
    if(0 != _ring->isDynamic)
    {
        free(_ring->dataBuf);
        _ring->dataBuf = NULL;
    }

    free(_ring);
}

bool ring_fifo_is_full(ring_fifo_t *_ring)
{
    switch(_ring->type)
    {
        case RF_TYPE_FRAME:
        /*不适用*/
        return false;
        case RF_TYPE_STREAM:
        if(1 > _ring->size - (_ring->tail - _ring->head)) { return true; }
    }

    return false;
}

bool ring_fifo_is_empty(ring_fifo_t *_ring)
{
    if(_ring->tail == _ring->head) { return true; }

    return false;
}

uint32_t ring_fifo_get_size(ring_fifo_t *_ring)
{
    return _ring->size;
}
