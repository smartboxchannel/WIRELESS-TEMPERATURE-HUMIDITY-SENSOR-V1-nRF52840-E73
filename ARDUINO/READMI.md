## This works on MySensors

www.mysensors.org

For external interrupts to work, you must add \_\_attribute\_\_ ((weak)) in the Winterrups.c file before the void GPIOTE_IRQHandler () function.

File path - %LOCALAPPDATA%\Arduino15\packages\sandeepmistry\hardware\nRF5\0.6.0\cores\nRF5
