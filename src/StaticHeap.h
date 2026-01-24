#pragma once

#include "Debug.h"
#include <stdint.h>
#include <memory>
#include <cstdlib>
#include <cstring>

/// @brief Define a statically-allocated local heap
/// @tparam N Size of the heap
// template <int N>
class StaticHeap {
public:
    StaticHeap(size_t n) : heap(nullptr), heap_size(n) { 
        // heap = new uint8_t(n);
        // if (heap == nullptr) {
        //     Debug.println("ERROR: Could not allocate local heap");
        //     return;
        // }
        // reset();
    }

    void reset() {
        if (heap == nullptr) {
            Debug.printf("INIT LOCAL HEAP: %d bytes\n", heap_size);
            heap = (uint8_t*)(::calloc(1, heap_size));
            if (heap == nullptr) {
                Debug.println("ERROR: Could not allocate local heap");
                return;
            }
        }

        head = reinterpret_cast<Block*>(heap);
        head->size = heap_size - sizeof(Block);
        head->free = true;
        head->next = nullptr;
    }

    void* calloc(std::size_t n, std::size_t size) {
        if (!head)
            reset();

        // Debug.printf("local calloc: %d %d\n", n, size);

        std::size_t total = align_up(n * size);
        Block* curr = head;

        while (curr) {
            if (curr->free && curr->size >= total) {
                if (curr->size >= total + sizeof(Block) + alignof(std::max_align_t)) {
                    auto* split = reinterpret_cast<Block*>(
                        reinterpret_cast<uint8_t*>(curr + 1) + total
                    );
                    split->size = curr->size - total - sizeof(Block);
                    split->free = true;
                    split->next = curr->next;

                    curr->next = split;
                    curr->size = total;
                }

                curr->free = false;
                void* user = curr + 1;
                memset(user, 0, total);
                return user;
            }
            curr = curr->next;
        }

        Debug.printf("ERROR: Could not alloc %d bytes\n", n * size);
        return nullptr;
    }

    void free(void* ptr) {
        if (!ptr || !head)
            return;

        auto* block = reinterpret_cast<Block*>(ptr) - 1;
        block->free = true;

        // coalesce adjacent free blocks
        Block* curr = head;
        while (curr && curr->next) {
            if (curr->free && curr->next->free) {
                curr->size += sizeof(Block) + curr->next->size;
                curr->next = curr->next->next;
            } else {
                curr = curr->next;
            }
        }
    }

private:
    struct alignas(std::max_align_t) Block {
        size_t size;
        bool free;
        Block* next;
    };

    // const size_t HEAP_SIZE = N;
    // alignas(std::max_align_t) uint8_t heap[N];
    uint8_t* heap = nullptr;
    const size_t heap_size;

    Block* head = nullptr;

    size_t align_up(size_t n) {
        constexpr size_t a = alignof(std::max_align_t);
        return (n + a - 1) & ~(a - 1);
    }
};


