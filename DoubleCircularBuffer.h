#ifndef DOUBLECIRCULARBUFFER_H
#define DOUBLECIRCULARBUFFER_H

#include <algorithm>

class DoubleCircularBuffer{
public:
    static const int CAPACITY = 64;
    float bufferA[CAPACITY] = {};
    float bufferB[CAPACITY] = {};
    float* writer = bufferA;
    float* reader = bufferB;
    int writeHead = 0;
    int readHead = 0;
    int writeCount = 0;
    
    bool read(float& out){
        if(readHead >= writeCount){
            return false;
        }
        out = reader[readHead];
        readHead++;
        return true;
    }

    bool write(float val){
        if(writeHead >= CAPACITY){
            return false;
        }
        writer[writeHead] = val;
        writeHead++;
        return true;
    }

    void swapBuffers(){
        writeCount = writeHead;
        writeHead = 0;
        readHead = 0;
        std::swap(writer, reader);
    }

};

#endif