#ifndef USARTBOOTLOADER_H
#define USARTBOOTLOADER_H

#define USART_BAUDRATE 19200
#define DEVICE_RESET_WAIT_MS 200
#define DEVICE_BOOTLOADER_COMMANDS_LENGTH 11
#include "mbed.h"
#include "EventQueue2.h"
//#include <map>
#include "Log2.h"

typedef enum BootloaderCommand_t {
    NoLastCommand = -1,
    MagicKey = 0x7F,
    ACK = 0x79,
    NACK = 0x1F
} BootloaderCommand;

typedef enum BootloaderCommandIndex_t {
    // 0x00
    Get,
    // 0x01
    GetVersionAndRPS,
    // 0x02
    GetID,
    // 0x11
    ReadMemory,
    // 0x21
    Go,
    // 0x31
    WriteMemory,
    // 0x43 or 0x44
    EraseMemory,
    // 0x63
    WriteProtect,
    // 0x73
    WriteUnprotect,
    // 0x82
    ReadoutProtect,
    // 0x92
    ReadoutUnprotect
} BootloaderCommandIndex;

typedef enum BootloaderCompletionStatus_t {
    TargetFlashed,
    TargetNotFlashed,
    StatusUpdate
} BootloaderCompletionStatus;

typedef struct BootloaderCompletionState_t {
    const enum BootloaderCompletionStatus_t status;
    const char* message;
} BootloaderCompletionState;

class USARTBootloader;
typedef void (USARTBootloader::*CommandHandler)(void);

class USARTBootloader {
private:
    Log2 logger;

    int lastCommandSent;
    Thread eventThread;
    EventQueue2 queue;
    mbed::DigitalOut boot0;
    mbed::DigitalInOut nrst;
    mbed::UARTSerial usart;
    
    const char* targetFilename;
    Callback<void(const BootloaderCompletionState)> cbComplete;

    /**
     * Maps an index (of type BootloaderCommandIndex) to a command ID.
     */
    uint8_t commandMap[DEVICE_BOOTLOADER_COMMANDS_LENGTH];

    //typedef void (*commandHandlerMap) (void);
    CommandHandler commandHandlers[DEVICE_BOOTLOADER_COMMANDS_LENGTH];

    int resetCount;


    void sendCallback(const BootloaderCompletionStatus status, const char* message) {
        if (this->cbComplete) {
            BootloaderCompletionState state = {
                status,
                message
            };

            this->cbComplete(state);
        }
    }
    void initializeCommandHandlers() {
        commandMap[Get] = 0x00; // the value is ALWAYS 0x00 for the Get command

        commandHandlers[Get] = &USARTBootloader::handleGet;
        commandHandlers[GetVersionAndRPS] = &USARTBootloader::handleGetVersionAndRPS;
        commandHandlers[GetID] = &USARTBootloader::handleGetID;
        commandHandlers[ReadMemory] = &USARTBootloader::handleReadMemory;
        commandHandlers[Go] = &USARTBootloader::handleGo;
        commandHandlers[WriteMemory] = &USARTBootloader::handleWriteMemory;
        commandHandlers[EraseMemory] = &USARTBootloader::handleEraseMemory;
        commandHandlers[WriteProtect] = &USARTBootloader::handleWriteProtect;
        commandHandlers[ReadoutProtect] = &USARTBootloader::handleReadoutProtect;
        commandHandlers[ReadoutUnprotect] = &USARTBootloader::handleReadoutUnprotect;
    }

    void handleGet();
    void handleGetVersionAndRPS();
    void handleGetID();
    void handleReadMemory();
    void handleGo();
    void handleWriteMemory();
    void handleEraseMemory();
    void handleWriteProtect();
    void handleWriteUnprotect();
    void handleReadoutProtect();
    void handleReadoutUnprotect();

public:
    USARTBootloader(PinName _boot0, PinName _nrst, PinName _tx, PinName _rx)
        : logger("USARTBootloader"), lastCommandSent(-1), queue(32 * EVENTS_EVENT_SIZE),
        boot0(_boot0), nrst(_nrst), usart(_tx, _rx, USART_BAUDRATE),
        resetCount(0)
          
    {
        logger.newline().write("initializing...");
        initializeCommandHandlers();
        /*
         * The format is 8 bits, even parity, and stop=1
         * It must be set to blocking, or else it can lead to a weird condition where shit happens during a read/write.
         */
        usart.set_format(8, UARTSerial::Even, 1);
        usart.set_blocking(true);
        usart.sigio(callback(this, &USARTBootloader::handleIOEvent));

        this->eventThread.start(
            callback(&this->queue, &EventQueue::dispatch_forever)
        );

        this->initializeSlaveDevice();
    }
    ~USARTBootloader() {
        this->destroy();
    }

    void destroy() {
        this->queue.break_dispatch();
        this->eventThread.terminate();
        int closeStatus = this->usart.close();

        if (closeStatus) {
            logger.write("failed to close usart, status=%d", closeStatus);
        }
        if (cbComplete) {
            sendCallback(StatusUpdate, "USARTBootloader instance destroyed");
        }
    }
    bool flashSlaveDevice(const char* filename, Callback<void(enum BootloaderCompletionStatus_t, const char *)> callback) {
        
        if ((!filename || !strlen(filename))) {
            sendCallback(TargetNotFlashed, "no filename passed");
            return false;
        }

        return true;
    }

    void initializeSlaveDevice() {
        logger.write("set boot0, reset; trying to load into bootloader...");
        this->boot0.write(1);

        if (this->resetSlave()) {
            logger.write("reset=OK, starting bootloader sequence");
            this->usart_putc(MagicKey);
            logger.write("should be within bootloader now");
        } else {
            logger.write("reset=fail");
            sendCallback(TargetNotFlashed, "device failed to reset");
        }
    }

    bool resetSlave() {
        nrst.input();
        if (nrst.read() == 0) {
            // should throw some sort of exception here
            logger.write("reset failed; nrst is low!");
            return false;
        }

        logger.write("resetting slave... ");
        nrst.output();

        if (!nrst.is_connected()) {
            logger.write("nrst not connected");
            return false;
        }

        this->nrst.write(0);
        wait_ms(1);
        this->nrst.write(1);
        logger.write("reset #%d done!", ++this->resetCount);
        wait_ms(DEVICE_RESET_WAIT_MS);

        return true;
    }

    void handleIOEvent() {
        // DO NOT print from this function. It's within an ISR and it will seriously mess things up.
        // mbed will allow you to do pc.printf() but do NOT do that.
        short current_events = this->usart.poll(0);
        
        if (current_events & POLLIN) {
            // https://github.com/ARMmbed/mbed-os/blob/master/drivers/UARTSerial.cpp#L126
            queue.call(callback(this, &USARTBootloader::handleQueueEvent));
        }
    }

    void handleQueueEvent() {
        size_t size;
        uint8_t cmd = usart_getc(size);

        if (cmd == 0x00 || cmd == 0xC0 || this->resetCount == 0 || cmd == 0xFF) {

            // usually a parity bit error, but the underlying class doesn't detect it
            logger.write("invalid packet (%X)", cmd);
            /*
                was prev doing if cmd == 0x00 || cmd == 0xc0
                then just do uart_getc again and check that...
                need more elegant way to do this
            */
        }
        
        switch(cmd) {
            case ACK:
                logger.write(">> START ACK >>>>>>>>>>>>>>");
                handleACK();
                logger.write("<< DONE ACK <<<<<<<<<<<<<<<");
                break;
            case NACK:
                logger.write("NACK. TODO: handle this elegantly");
                break;
            default:
                logger.write("<!> invalid command: %x", cmd);
                
                if (!queue.has_next() && (this->usart.poll(0) & 1)) {
                    logger.write("\t-> there seems to be more in the rxfifo and an empty equeue");
                    logger.write("\t\t-> will re-enqueue another check");
                    queue.call(callback(this, &USARTBootloader::handleQueueEvent));
                }
                break;
        }
    }

    
    size_t usart_putc(uint8_t b) {
        size_t written = usart.write(&b, 1);
        usart.sync();

        return written;
    }

    uint8_t usart_getc(size_t &out) {
        // https://github.com/ARMmbed/mbed-os/blob/master/drivers/UARTSerial.cpp#L259
        
        uint8_t tmp;
        out = usart.read(&tmp, 1);

        return tmp;
    }

    void usart_writeBootloaderCommand(const BootloaderCommandIndex command) {
        // add mutex? should be fine w/o though
        uint8_t b0 = command == Get ? 0x00 : commandMap[command]; // note: Get is always 0x00
        uint8_t tx[2] = { b0, (uint8_t)(0xFF ^ b0) };
        lastCommandSent = b0;
        logger.write("\tWRITE cmd pair <%X, %X>", tx[0], tx[1]);
        this->usart.write(tx, 2);
        this->usart.sync();
    }
    
    void handleACK() {
        logger.write("\thandling ACK, after cmd(%x) in sequence", lastCommandSent);

        switch(lastCommandSent) {
            // first ACK
            case NoLastCommand: {
                logger.write("\tnext command = get version (0x00)");

                if (resetCount > 1) {
                    logger.write("\thalting until I get this figured out");
                } else {
                    usart_writeBootloaderCommand(Get);
                }
            }
            break;

            default: {
                uint8_t* commandMapPtr = std::find(
                    commandMap,
                    commandMap + DEVICE_BOOTLOADER_COMMANDS_LENGTH,
                    lastCommandSent
                );

                if (commandMapPtr == commandMap + DEVICE_BOOTLOADER_COMMANDS_LENGTH) {
                    logger.write("<!> no handler found for command %x!", lastCommandSent);
                } else {
                    int commandIndex = commandMapPtr - commandMap;
                    CommandHandler handler = commandHandlers[commandIndex];
                    (this->*handler)();
                }
            }
            break;


            case 0x73: {
                size_t s;
                logger.write("\twrite protect? acked?=%x\n", this->usart_getc(s));
            }
            break;
        }
    }
};

#endif