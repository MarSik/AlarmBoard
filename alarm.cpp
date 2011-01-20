#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define LEDDATA PORTB
#define XLAT 0
#define SCLK 7
#define SOUT 6
#define SIN 5
#define BLANK 1

#define LEDCTRL PORTD
#define VPRG 6
#define DCPRG 4
#define RXD 0

#define sbi(port, bit) (port) |= _BV((bit))
#define cbi(port, bit) (port) &= ~(_BV((bit)))

uint16_t leds[2][16] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

volatile struct _ledstatus {
    uint8_t active:1;
    uint8_t changed:1;
} ledstatus = {0, 0};

volatile struct _serialstate {
    uint8_t nothing:1;
    uint8_t state:2; // 0 - wait for command, 1 - for addr, 2 - for brightness H, 3 - waiting for brightness L
    uint16_t brightness:12;
    uint8_t addr:4;
} serialStatus = {1, 0, 0, 0};

void uartTx(uint8_t ch)
{
    while(!(UCSRA & _BV(UDRE)));
    UDR = ch;
}

int main(void)
{
    // enable interrupts
    sbi(SREG, 7);

    // set port direction to output, except receive ports
    DDRB = 0xff & ~_BV(SOUT);
    DDRD = 0xff & ~_BV(RXD);

    // set grayscale register mode
    sbi(LEDCTRL, DCPRG);
    cbi(LEDCTRL, VPRG);

    // set timer1 to control BLANK signal
    TCCR1A = 0; // no external timer output, CTC mode
    TCCR1B = _BV(WGM12) | _BV(CS10); // internal clock, no prescaler, CTC, TOP is OCR1A
    
    OCR1AH = 0x0f;
    OCR1AL = 0xff; // 4096 steps

    TIMSK = _BV(OCIE1A); // INT when OCR1A is matched

    // set USART to 9600baud, 8N1, RX interrupt
    UBRRH = 0;
    UBRRL = 51;
    UCSRC = _BV(USBS) | _BV(UCSZ1) | _BV(UCSZ0);
    UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);

    while(1){
        if(!serialStatus.nothing){
            leds[ledstatus.active][serialStatus.addr] = serialStatus.brightness;
            serialStatus.nothing = 1;
        }
    }
}

inline void transferLedData(uint16_t br)
{
    int8_t counter;
    for(counter=11; counter>=0; counter--){
        // feed msb first
        LEDDATA = (LEDDATA & ~_BV(SIN)) | (((br >> counter) & 1) << SIN);

        // cycle clk pin
        sbi(PINB, SCLK);
        sbi(PINB, SCLK);
    }
}

ISR(USART_RX_vect)
{
    uint8_t data = UDR;

    switch(serialStatus.state){
    case 0:
        if(data==0xff) serialStatus.state++;
        else if(data==0xfc) ledstatus.changed = 1;
        else if(data==0xfd) ledstatus.active ^= 1;
        else if(data==0xf0) memset(leds[ledstatus.active], 0, 64);         
        break;
    case 1:
        serialStatus.addr = data & 0xf;
        serialStatus.state++;
        break;
    case 2:
        serialStatus.brightness = (data & 0xf) << 8;
        serialStatus.state++;
        break;
    case 3:
        serialStatus.brightness += data;
        serialStatus.state = 0;
        serialStatus.nothing = 0;
        break;
    }
}

ISR(TIMER1_COMPA_vect)
{
    // cycle blank pin
    sbi(PINB, BLANK);
    sbi(PINB, BLANK);

    if(ledstatus.changed){
        // commit data
        ledstatus.changed = 0;

        int i;
        for(i=0; i<16; i++) transferLedData(leds[ledstatus.active][i]);

        // latch data to driver
        sbi(PINB, XLAT);
        sbi(PINB, XLAT);
    }
}
