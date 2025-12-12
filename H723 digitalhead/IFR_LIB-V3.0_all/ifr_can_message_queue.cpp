#include "ifr_can_message_queue.h"
#include "string.h" // 包含 memcpy

uint8_t MessageQueue::memory_pool[MEMORY_POOL_SIZE] = {0};
size_t MessageQueue::memory_offset = 0;

// 构造函数
MessageQueue::MessageQueue() : front(0), rear(0), count(0) {}

// 内存分配
void* MessageQueue::my_malloc(size_t size) {
    if (memory_offset + size > MEMORY_POOL_SIZE) {
        return NULL; // 内存不足
    }
    void* ptr = &memory_pool[memory_offset];
    memory_offset += size;
    return ptr;
}

// 内存释放
void MessageQueue::my_free(size_t size) {
    if (memory_offset >= size) {
        memory_offset -= size;
    }
}

// 添加或更新消息
uint8_t MessageQueue::addOrUpdate(const FDCAN_TxHeaderTypeDef& header, const uint8_t* data, uint32_t size, uint32_t modeId) {
    // 搜索最近的同模式 ID 消息
    for (int i = 0; i < count; ++i) {
        int index = (front + i) % QUEUE_MAX_SIZE;
        if (queue[index].header.Identifier == header.Identifier && queue[index].modeId == modeId) {
            // 找到相同模式 ID 的消息，更新其内容
            if (size > queue[index].size) {
                // 如果新数据大小超出已有数据大小，重新分配内存
                my_free(queue[index].size);
                queue[index].data = (uint8_t*)my_malloc(size);
                if (queue[index].data == NULL) {
                    return 0; // 内存分配失败
                }
            }
            queue[index].size = size;
            memcpy(queue[index].data, data, size);
            return 1; // 更新成功
        }
    }

    // 如果不存在相同模式 ID 消息，则加入队列
    if (isFull()) {
        return 0; // 队列已满
    }
    queue[rear].header = header;
    queue[rear].size = size;
    queue[rear].data = (uint8_t*)my_malloc(size);
    queue[rear].modeId = modeId;

    if (queue[rear].data == NULL) {
        return 0; // 内存分配失败
    }
    memcpy(queue[rear].data, data, size);

    rear = (rear + 1) % QUEUE_MAX_SIZE; // 更新队尾索引
    count++;
    return 2; // 入队成功
}

// 出队
uint8_t MessageQueue::dequeue(FDCAN_TxHeaderTypeDef& header, uint8_t*& data, uint32_t& size, uint32_t& modeId) {
    if (isEmpty()) {
        return 0; // 队列为空
    }

    header = queue[front].header;
    size = queue[front].size;
    data = queue[front].data;
    modeId = queue[front].modeId;

    my_free(size);

    front = (front + 1) % QUEUE_MAX_SIZE; // 更新队头索引
    count--;
    return 1;
}

// 检测是否为空
uint8_t MessageQueue::isEmpty() const {
    return count == 0;
}

// 检测是否已满
uint8_t MessageQueue::isFull() const {
    return count == QUEUE_MAX_SIZE;
}
