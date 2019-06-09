#ifndef LOG2_H
#define LOG2_H
#include "mbed.h"

#include <cstdio>
#include <cstdarg>
#include <string>

#define STRING_STACK_LIMIT    120

/**
 * An extension of RawSerial, to include a vprintf function.
 * NOTE: while the current stable mbed does not contain this
 * function, it is included in the current git master of mbed
 */
class RawSerial2 : public RawSerial {
public:
    RawSerial2(PinName tx, PinName rx)
        : RawSerial(tx, rx)
    { }

    int vprintf(const char *format, std::va_list arg)
    {
        lock();
        // ARMCC microlib does not properly handle a size of 0.
        // As a workaround supply a dummy buffer with a size of 1.
        char dummy_buf[1];
        int len = vsnprintf(dummy_buf, sizeof(dummy_buf), format, arg);
        if (len < STRING_STACK_LIMIT) {
            char temp[STRING_STACK_LIMIT];
            vsprintf(temp, format, arg);
            puts(temp);
        } else {
            char *temp = new char[len + 1];
            vsprintf(temp, format, arg);
            puts(temp);
            delete[] temp;
        }
        unlock();
        return len;
    }
};

/**
 * A kinda shitty logging class with a mutex
 * 
 * @example
 *      Log2 log("MyComponent");
 *      log.write("test");
 */
class Log2 {
public:
    Log2(const char* _tag)
        : tag(_tag), pc(USBTX, USBRX), useMutex(true)
    {}
    Log2(const char* _tag, bool _useMutex)
        : tag(_tag), pc(USBTX, USBRX), useMutex(_useMutex)
    {}

    Log2& write(const char* format, ...) {
        if (this->useMutex) {
            this->mutex.lock();
        }
        va_list ap;
        va_start(ap, format);

        pc.printf("[%s] ", tag);
        pc.vprintf(format, ap);
        pc.putc('\n');
        va_end(ap);
        if (this->useMutex) {
            this->mutex.unlock();
        }

        return *this;
    }

    Log2& newline() {
        if (this->useMutex) {
            this->mutex.lock();
        }
        pc.putc('\n');
        if (this->useMutex) {
            this->mutex.unlock();
        }

        return *this;
    }
private:
    const char *tag;
    RawSerial2 pc;
    Mutex mutex;
    bool useMutex;
};

#endif