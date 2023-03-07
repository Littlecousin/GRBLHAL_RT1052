#ifndef __RING_FIFO_H
#define __RING_FIFO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

/*ring state*/
#define RF_SUCCESS                  0    /*无错误*/
#define RF_MALLOC_FAILED            1    /*申请内存失败*/
#define RF_INVALID_PARAMETER        2    /*无效的参数*/
#define RF_FULL                     3    /*缓冲区满*/
#define RF_EMPTY                    4    /*缓冲区空*/
#define RF_READ_BUFFER_TOO_SMALL    5    /*读缓冲太小*/

/*ring type*/
enum ring_fifo_type {
    RF_TYPE_FRAME,//帧模式
    RF_TYPE_STREAM//流模式
};

/*环形缓冲区相关结构体*/
typedef struct _ring_fifo_t {
    volatile uint32_t   head;           /*缓冲区头部*/
    volatile uint32_t   tail;           /*缓冲区尾部*/
    uint32_t            size;           /*缓冲区的大小*/
    uint8_t             *dataBuf;       /*指向缓冲区的指针*/
    uint32_t            isDynamic;      /*是否使用了动态内存*/
    enum ring_fifo_type type;           /*fifo的类型*/
}ring_fifo_t;

/**
* @brief    初始化环形缓冲区
* @param    __ring  作为fifo句柄返回
* @param    pBuf    存储缓冲区，如果为NULL，则使用堆内存自动分配
* @param    size    存储缓冲区长度（如果值不是2的幂次方，则会自动对齐到2的幂次方）
* @param    type    fifo的类型
* @retval   执行状态
*/
uint32_t ring_fifo_init(ring_fifo_t **__ring, uint8_t *pBuf, uint32_t size, enum ring_fifo_type type);

/**
* @brief    写入数据到环形缓冲区
* @param    _ring   对应fifo句柄
* @param    pBuf    待写入数据缓冲区
* @param    len     待写入数据长度
* @retval   执行状态
*/
uint32_t ring_fifo_write(ring_fifo_t *_ring, const uint8_t *pBuf, uint16_t len);

/**
* @brief    从环形缓冲区读出数据
* @param    _ring   对应fifo句柄
* @param    pBuf    读出数据缓冲区
* @param    len     待读出数据缓冲区长度（type == RF_TYPE_FRAME），欲读出的数据长度（type == RF_TYPE_STREAM）
* @param    retLen  实际读出数据长度
* @retval   执行状态
*/
uint32_t ring_fifo_read(ring_fifo_t *_ring, uint8_t *pBuf, uint16_t len, uint16_t *retLen);

/**
* @brief    销毁对应fifo
* @param    _ring   对应fifo句柄
* @retval   无
*/
void ring_fifo_destroy(ring_fifo_t *_ring);

/**
* @brief    判满（注意：只适用于流式fifo）
* @param    _ring   对应fifo句柄
* @retval   true：满，false：未满
*/
bool ring_fifo_is_full(ring_fifo_t *_ring);

/**
* @brief    判空
* @param    _ring   对应fifo句柄
* @retval   true：空，false：非空
*/
bool ring_fifo_is_empty(ring_fifo_t *_ring);

/**
* @brief    获取fifo大小
* @param    _ring   对应fifo句柄
* @retval   fifo大小
*/
uint32_t ring_fifo_get_size(ring_fifo_t *_ring);
uint32_t ring_fifo_get_available_size(ring_fifo_t *_ring);
#ifdef __cplusplus
}
#endif

#endif/*__RING_FIFO_H*/
