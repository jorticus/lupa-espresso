#pragma once
#include <Stream.h>
#include <stdint.h>

#include "vfs_api.h"

using namespace fs;

class DATAFSImpl : public VFSImpl
{
public:
    DATAFSImpl();
    virtual ~DATAFSImpl() { }
    virtual bool exists(const char* path) { return true; }
};

class DATAFS : public FS
{
public:
    DATAFS() : FS(FSImplPtr(new DATAFSImpl())) { }
    ~DATAFS() { }
    bool begin() { return true; }
};

class DataFileImpl : public FileImpl
{
public:

};


class DataStream : public Stream
{
public:
    DataStream(const uint8_t* data, size_t len) :
        data(data), len(len), _i(0)
    { }
    virtual ~DataStream() {

    }

    int available() override {
        return len;
    }
    int read() override {
        if (_i < this->len) {
            return this->data[_i++];
        }
        return -1;
    }
    int peek() override {
        if (_i < this->len) {
            return this->data[_i];
        }
        return -1;
    }

    size_t readBytes(char* buffer, size_t len) override {
        if (len > this->len)
            len = this->len;

        memcpy(buffer, this->data, len);
        return len;
    }

    size_t write(uint8_t) override { return 0; }
    size_t write(const uint8_t *buffer, size_t size) override { return 0; }

private:
    const uint8_t* data;
    size_t len;
    size_t _i;
};