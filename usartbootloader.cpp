#include "usartbootloader.h"

void USARTBootloader::handleGet() {
    logger.write("\treceived get?!!?!!!?!");
}
void USARTBootloader::handleGetVersionAndRPS() {
    logger.write("\treceived GetVersionAndRPS");
    size_t s;
    uint8_t v = this->usart_getc(s),
        opt1 = this->usart_getc(s),
        opt2 = this->usart_getc(s);
    logger.write("\tv=%x, o[%x, %x]", v, opt1, opt2);
    uint8_t ack2 = this->usart_getc(s);

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
    
    size_t s;
    uint8_t ack2 = this->usart_getc(s);

    if (ack2 == ACK) {
        logger.write("\trecv'd second ack!");

        usart_writeBootloaderCommand(WriteUnprotect);
    } else {
        logger.write("<!> no second ack for 0x92");
        sendCallback(TargetNotFlashed, "missing second ACK from 0x92");
    }
}