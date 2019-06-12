#ifndef USARTBOOTLOADER_H
#define USARTBOOTLOADER_H

#define DEVICE_USART_BAUDRATE 19200
#define DEVICE_INVALID_VERSION 0xFF

#define STM32L4X6_FLASH_BANK1_ADDR_START 0x08000000
#define STM32L4X6_FLASH_BANK1_ADDR_END   0x0807FFFF

#define STM32L4X6_FLASH_BANK2_ADDR_START 0x08080000
#define STM32L4X6_FLASH_BANK2_ADDR_END 0x080FFFFF

/**
 * It should be at least 500ms.
 * Not sure if there is a duration in the documentation, but from testing,
 * I've found it takes around 500ms on average.
 */ 
#define DEVICE_RESET_WAIT_MS 500

/**
 * The bootloader should only support 11 commands, but I think for older
 * bootloader versions, it may only support less.
 */  
#define DEVICE_BOOTLOADER_COMMANDS_LENGTH 11

#include <cstdio>
#include "mbed.h"
#include "EventQueue2.h"
#include "Log2.h"

/**
 * I should come up with a better name for these enums. 
 */ 
typedef enum BootloaderCommand_t {
    NoLastCommand = 0xFF,
    MagicKey      = 0x7F,
    ACK           = 0x79,
    NACK          = 0x1F
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

    uint8_t lastCommandSent;
    Thread eventThread;
    EventQueue2 queue;
    mbed::DigitalOut boot0;
    mbed::DigitalInOut nrst;
    mbed::UARTSerial usart;
    
    /**
     * TODO
     */
    const char* targetFilename;

    /**
     * Callback for when either the device is flashed or errors.
     * Maybe should re-think what this does.
     */
    Callback<void(const BootloaderCompletionState)> cbComplete;

    /**
     * Maps an index (of type BootloaderCommandIndex) to a command ID.
     * This basically only exits for additional bootloader versions and
     * to support several variations of the erase command.
     */
    uint8_t commandMap[DEVICE_BOOTLOADER_COMMANDS_LENGTH];

    /**
     * Maps a command index to a function pointer:
     * commandHandlers[Get] = &someFunction;
     */
    CommandHandler commandHandlers[DEVICE_BOOTLOADER_COMMANDS_LENGTH];

    /**
     * Container for holding information about the device.
     */
    typedef struct DeviceInformation_t {
        uint8_t bootloaderVersion;
        uint8_t resetCount;
    } DeviceInformation;

    DeviceInformation deviceInformation;

    
    bool readFlashRegister32(const uint32_t address, uint32_t &out);

    void sendCallback(const BootloaderCompletionStatus status, const char* message);
    void initializeCommandHandlers();

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
    
    void handleIOEvent();
    void destroy();
    void handleReadRequested();

    void test();
public:
    USARTBootloader(PinName _boot0, PinName _nrst, PinName _tx, PinName _rx)
        : logger("USARTBootloader"), lastCommandSent(NoLastCommand), queue(32 * EVENTS_EVENT_SIZE),
          boot0(_boot0), nrst(_nrst), usart(_tx, _rx, DEVICE_USART_BAUDRATE)
    {
        logger.newline().write("initializing...");
        initializeCommandHandlers();

        deviceInformation.bootloaderVersion = DEVICE_INVALID_VERSION;
        deviceInformation.resetCount = 0;

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

    /**
     * TODO
     */
    bool flashSlaveDevice(const char* filename, Callback<void(enum BootloaderCompletionStatus_t, const char *)> callback) {
        if ((!filename || !strlen(filename))) {
            sendCallback(TargetNotFlashed, "no filename passed");
            return false;
        }

        // TODO
        return true;
    }

    /**
     * Initialize the slave device for flashing; reboots the device into its bootloader
     */
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

    /**
     * Resets the slave by toggling NRST
     */ 
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
        logger.write("reset #%d done!", ++this->deviceInformation.resetCount);
        wait_ms(DEVICE_RESET_WAIT_MS);

        return true;
    }
    
    /**
     * Puts a single byte into the txfifo and ensures it sends.
     */ 
    size_t usart_putc(uint8_t b) {
        size_t written = usart.write(&b, 1);

        usart.sync();
        return written;
    }


    /**
     * Gets a single byte from the rxfifo.
     */
    uint8_t usart_getc() {
        uint8_t tmp;

        usart.read(&tmp, 1);

        return tmp;
    }

    /**
     * Gets a single byte from the rxfifo.
     * 
     * Related: https://github.com/ARMmbed/mbed-os/blob/master/drivers/UARTSerial.cpp#L259
     */ 
    uint8_t usart_getc(size_t &out) {
        uint8_t tmp;

        out = usart.read(&tmp, 1);

        return tmp;
    }


    /**
     * Transmits a command to the slave device
     */
    void usart_writeBootloaderCommand(const BootloaderCommandIndex command) {
        // add mutex? should be fine w/o though
        uint8_t b0 = (command == Get) ? 0x00 : commandMap[command];
        uint8_t tx[2] = { b0, (uint8_t)(0xFF ^ b0) };

        lastCommandSent = b0;
        logger.write("\tWRITE cmd pair <%X, %X>", tx[0], tx[1]);

        this->usart.write(tx, 2);
        this->usart.sync();
    }
    
    /**
     * Handle an ACK sent by the slave device.
     */
    void handleACK() {
        logger.write("\thandling ACK for %X", lastCommandSent);

        // if this is the first ACK after a reset
        if (lastCommandSent == NoLastCommand) {
            logger.write("\thandshake: OK");

            if (deviceInformation.resetCount > 2) {
                logger.write("HMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");
                test();
                //wait(osWaitForever);
            }
            else if (deviceInformation.resetCount > 1) {
                // unsure if this is even needed?
                usart_writeBootloaderCommand(WriteUnprotect);
                //test();
            } else {
                logger.write("\tfirst reboot -> requesting Get...");

                usart_writeBootloaderCommand(Get);
            }

            return;
        }

        // basically: find which handler to use
        uint8_t* commandMapPtr = std::find(
            commandMap,
            commandMap + DEVICE_BOOTLOADER_COMMANDS_LENGTH,
            lastCommandSent
        );

        if (commandMapPtr == commandMap + DEVICE_BOOTLOADER_COMMANDS_LENGTH) {
            // this doesn't work
            // need to debug and see why it doesn't work :/
            // may need a sizeof() in there somewhere
            logger.write("<!> no handler found for command %x!", lastCommandSent);
        } else {
            int commandIndex = commandMapPtr - commandMap;
            CommandHandler handler = commandHandlers[commandIndex];
            (this->*handler)();
        }   
    
    }
};

#endif