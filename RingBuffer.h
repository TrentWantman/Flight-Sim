#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <iostream>

class RingBuffer{
public:
    int capacity_;
    float* buffer;
    int head;
    int tail;
    int count_;

    RingBuffer(const int&  capacity){
        buffer = new float[capacity];
        capacity_ = capacity;
        for (int i = 0; i < capacity_; i++){
            buffer[i] = 0;
        }
        head = 0; count_=0; tail = 0;
    }

    ~RingBuffer(){
        delete[] buffer;
        buffer = nullptr;
        capacity_ = 0;
    }

    bool isEmpty() { return count_ == 0; }
    bool isFull() { return count_ == capacity_; }

    void push(const float& value){
        if(isFull()){ head = (head+1) % capacity_;}
        else{count_++;}
        buffer[tail] = value;
        tail = (tail+1)% capacity_;
    }

    float pop(){
        if(isEmpty()){ std::cout << "Buffer is empty" << std::endl; return -1;}
        else{ count_--;}
        float temp = buffer[head];
        buffer[head] = 0;
        head = (head+1) % capacity_;
        return temp;
    }
    
};

#endif