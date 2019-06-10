#include "usartbootloader.h"

/**
 * Converts a 32-bit address to an uint8_t array with a checksum byte.
 * 
 * Returns the address to that array.
 */
uint8_t* addr2arrcs(uint32_t address, uint8_t arr[5]) {
    arr[0] = (uint8_t) ((address >> 24) & 0xFF);
    arr[1] = (uint8_t) ((address >> 16) & 0xFF);
    arr[2] = (uint8_t) ((address >> 8) & 0xFF);
    arr[3] = (uint8_t) ((address >> 0) & 0xFF);
    arr[4] = arr[0] ^ arr[1] ^ arr[2] ^ arr[3];

    return arr;
}
bool USARTBootloader::readFlashRegister32(const uint16_t offset, uint32_t &out) {
    logger.write("Reading flash register, offset@%X", offset);

    usart_writeBootloaderCommand(ReadMemory);

    uint32_t startAddress = STM32L4X6_FLASH_BANK1_REG_BASEADDR + offset;
    uint8_t data[5], ack = NACK;

    ack = usart_getc();
    if (ack != ACK) {
        logger.write("did not receive ACK after read mem command");
        return false;
    }

    // write address and ack:
    addr2arrcs(startAddress, data);
    usart.write(data, 5);
    
    ack = usart_getc();
    if (ack != ACK) {
        logger.write("did not receive ACK after writing address");
        return false;
    }

    // write N-1 bytes to read and ack:
    data[0] = 3; // N-1
    data[1] = 0xFF ^ 3;
    usart.write(data, 2);

    ack = usart_getc();
    if (ack != ACK) {
        logger.write("did not receive ACK after writing size");
        return false;
    }

    /*
        endian needs to be flipped:
            offset@0 => data=FFEFF8AA [AA, F8, EF, FF]
            production value = 0xFFEF F8AA
        base addr needs to be corrected
    */
    usart.read(data, 4);
    memcpy(&out, data, 4);

    logger.write("\t\toffset@%X => data=%X [%X, %X, %X, %X]", offset, out, data[0], data[1], data[2], data[3]);
    return true;
}
void USARTBootloader::test() {
    uint32_t value;
    readFlashRegister32(0x00 /* FLASH_ACR */, value);
    readFlashRegister32(0x10 /* FLASH_SR */, value);
    

}
void USARTBootloader::sendCallback(const BootloaderCompletionStatus status, const char* message) {
    if (this->cbComplete) {
        BootloaderCompletionState state = {
            status,
            message
        };

        this->cbComplete(state);
    }

    // TODO: call destroy
}


/**
 * Initializes the command handlers by mapping commands to function ptrs.
 */
void USARTBootloader::initializeCommandHandlers() {
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


/**
 * Blows things up
 */
void USARTBootloader::destroy() {
    this->queue.break_dispatch();
    this->eventThread.terminate();

    int closeStatus = this->usart.close();

    if (closeStatus) {
        logger.write("failed to close usart, status=%d", closeStatus);
    }

    sendCallback(StatusUpdate, "USARTBootloader instance destroyed");
}

/**
 * Handler for any type of IO signal.
 * 
 * NOTE: within an ISR!
 */ 
void USARTBootloader::handleIOEvent() {
    // DO NOT print from this function. It's within an ISR and it will seriously mess things up.
    // mbed will allow you to do pc.printf() but do NOT do that.
    short current_events = this->usart.poll(0);
    
    if (current_events & POLLIN) {
        // https://github.com/ARMmbed/mbed-os/blob/master/drivers/UARTSerial.cpp#L126
        queue.call(callback(this, &USARTBootloader::handleReadRequested));
    }
}

/**
 * Handler for an event in the queue.
 */
void USARTBootloader::handleReadRequested() {
    size_t size;
    uint8_t cmd = usart_getc(size);
    
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
            break;
    }

    if (!queue.has_next() && (this->usart.poll(0) & 1)) {
        logger.write(">> there seems to be more in the rxfifo and an empty equeue");
        logger.write("\t-> this is OK if the device just reset and it's reading garbage");
        logger.write("\t-> will re-enqueue another check");
        logger.write("<<");

        queue.call(callback(this, &USARTBootloader::handleReadRequested));
    }
}


void USARTBootloader::handleGet() {
    logger.write("\treceived Get!");

    uint8_t commandsLength = this->usart_getc();
    deviceInformation.bootloaderVersion = this->usart_getc();

    logger.write("\tbootloader version = %x", deviceInformation.bootloaderVersion);
    
    //TODO check to see if bootloader version is supported
    if (commandsLength != DEVICE_BOOTLOADER_COMMANDS_LENGTH) {
        logger.write(
            "\t<!> unepxected number of commands available: got %d should be %d",
            commandsLength,
            DEVICE_BOOTLOADER_COMMANDS_LENGTH
        );
        return;
    }

    // Read all 11 commands into the commandMap
    this->usart.read(this->commandMap, DEVICE_BOOTLOADER_COMMANDS_LENGTH);

    uint8_t ack2 = this->usart_getc();

    if (ack2 != ACK) {
        logger.write("\t<!> second ack is invalid for get!");
        return;
    }

    usart_writeBootloaderCommand(GetVersionAndRPS);
}

void USARTBootloader::handleGetVersionAndRPS() {
    logger.write("\treceived GetVersionAndRPS");

    uint8_t v = this->usart_getc(),
        opt1 = this->usart_getc(),
        opt2 = this->usart_getc(),
        ack2 = this->usart_getc();

    logger.write("\tv=%x, o[%x, %x]", v, opt1, opt2);

    if (ack2 != ACK) {
        logger.write("\t<!>unexpected non-ack2: %x", ack2);
        return;
    }
    usart_writeBootloaderCommand(ReadoutUnprotect);
}

void USARTBootloader::handleGetID() {
    logger.write("\treceived get ID");
}

void USARTBootloader::handleReadMemory() {
    logger.write("\treceived read memory");
}

void USARTBootloader::handleGo() {
    logger.write("\treceived go");
}

void USARTBootloader::handleWriteMemory() {
    logger.write("\treceived write memory");
}

void USARTBootloader::handleEraseMemory() {
    logger.write("\treceived erase memory");
}

void USARTBootloader::handleWriteProtect() {
    logger.write("\treceived write protect");
}

void USARTBootloader::handleWriteUnprotect() {
    logger.write("\treceived write unprotect");
}

void USARTBootloader::handleReadoutProtect() {
    logger.write("\treceived readout protect");
}

void USARTBootloader::handleReadoutUnprotect() {
    logger.write("\teceived first ack for 0x92, waiting for ack after reset...");
    uint8_t ack2 = this->usart_getc(); // this should be OK if it's blocking, right?

    if (ack2 == ACK) {
        logger.write("\trecv'd second ack!");
        logger.write("\tsleeping before sending WriteProtect");
        deviceInformation.resetCount++;
        wait_ms(DEVICE_RESET_WAIT_MS);

        lastCommandSent = NoLastCommand;
        this->usart_putc(MagicKey);
    } else {
        logger.write("<!> no second ack for 0x92");
        sendCallback(TargetNotFlashed, "missing second ACK from 0x92");
    }
}

