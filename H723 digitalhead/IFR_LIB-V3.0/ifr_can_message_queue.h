#ifndef IFR_CAN_MESSAGE_QUEUE_H
#define IFR_CAN_MESSAGE_QUEUE_H

#include "main.h"
#include <cstddef>
#include <cstdint>

#define QUEUE_MAX_SIZE 1       // 最大队列长度
#define MEMORY_POOL_SIZE 1   // 定义内存池大小

class MessageQueue {
public:
    struct Message {
        FDCAN_TxHeaderTypeDef header;
        uint8_t* data;
        uint32_t size;
        uint32_t modeId; // 模式 ID，用于区分不同类型的消息
    };

private:
    Message queue[QUEUE_MAX_SIZE]; // 消息队列
    int front;                     // 队头索引
    int rear;                      // 队尾索引
    int count;                     // 当前队列中的消息数量

    // 内存池
    static uint8_t memory_pool[MEMORY_POOL_SIZE];
    static size_t memory_offset;

    // 内存分配和释放
    static void* my_malloc(size_t size);
    static void my_free(size_t size);

public:
    MessageQueue();
    uint8_t addOrUpdate(const FDCAN_TxHeaderTypeDef& header, const uint8_t* data, uint32_t size, uint32_t modeId);
    uint8_t dequeue(FDCAN_TxHeaderTypeDef& header, uint8_t*& data, uint32_t& size, uint32_t& modeId);
    uint8_t isEmpty() const;
    uint8_t isFull() const;
};

#endif // IFR_CAN_MESSAGE_QUEUE_H
