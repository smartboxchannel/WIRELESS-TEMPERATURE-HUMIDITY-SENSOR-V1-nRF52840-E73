## This works on MySensors

www.mysensors.org

For external interrupts to work, you must add '''__attribute__ ((weak))''' in the Winterrups.c file before the void GPIOTE_IRQHandler () function.

File path - C:\Users\!!!USER NAME!!!\AppData\Local\Arduino15\packages\sandeepmistry\hardware\nRF5\0.6.0\cores\nRF5
