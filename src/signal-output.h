#ifndef _SIGNAL_OUTPUT_H_
#define _SIGNAL_OUTPUT_H_

#define SIG_OPTION_COM_ANODE  0x01

typedef void (*SignalOutputFunc)(uint8_t, uint8_t, uint8_t, uint8_t);

void initSignalOutputs();
void setSignalS1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca);
void setSignalS2(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca);
void setSignalS3(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca);
void setSignalS4(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca);

#endif
