#ifndef CYCLIC_BUFFER_H
#define CYCLICBUFFER_H
#include <cstdio>
#include <memory>
#include <mutex>
#include "opencv2/imgproc/imgproc.hpp"
template <class T>
class circular_buffer{
public:
    circular_buffer(size_t size) :
        buf_(std::unique_ptr<T[]>(new T[size])),
        size_(size)
    {
        //empty constructor
    }

    void put(T item);
    T get(void);

    void reset(void)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_;
    }

    bool empty(void);
    bool full(void);
    size_t size(void);

private:
    std::mutex mutex_;
    std::unique_ptr<T[]> buf_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t size_;
};
#endif
