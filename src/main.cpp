#include "mbed.h"

Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);

int main(void)
{
	// Flash on board led at 1Hz
	for(;;){
		led1 = !led1;
		pc.printf("Toggling LED1\n");
		wait(0.5);
	}
}
