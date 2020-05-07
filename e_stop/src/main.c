#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.c"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.c"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2

#define BUTTON_BASE GPIO_PORTF_BASE
#define BUTTON GPIO_PIN_4
#define BUTTON_INT GPIO_INT_PIN_4

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

static volatile bool stop = false;

void configure_UART() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}

//void set_sensor_state() {
//    uint8_t terminator = 0xFF;
//
//    while(true) {
//        uint8_t encoderPositionLSB = UARTgetc();
//        uint8_t encoderPositionMSB = UARTgetc();
//        terminator = UARTgetc();
//
//        if(terminator == 0) {
//            return encoderPositionLSB + (encoderPositionMSB << 8);
//        }
//    }
//}

void on_button_down() {
    uint32_t status = GPIOIntStatus(BUTTON_BASE, true);

    if(status & BUTTON_INT) {
        GPIOPinWrite(LED_BASE, BLUE_LED, BLUE_LED);
        stop = true;
    }

    GPIOIntClear(BUTTON_BASE, BUTTON);
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(LED_PERIPH);
    SysCtlDelay(3);

    configure_UART();

    GPIOPinTypeGPIOInput(BUTTON_BASE, BUTTON);
    GPIOPadConfigSet(BUTTON_BASE, BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(BUTTON_BASE, BUTTON);
    GPIOIntClear(BUTTON_BASE, BUTTON);
    GPIOIntTypeSet(BUTTON_BASE, BUTTON, GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOF, 0);
    GPIOIntRegister(BUTTON_BASE, on_button_down);
    GPIOIntEnable(BUTTON_BASE, BUTTON_INT);

    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED);

    while(1) {
        if (stop == true) {
            GPIOPinWrite(LED_BASE, RED_LED, RED_LED);
        }
    }
}
