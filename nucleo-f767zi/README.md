# STM32F767ZIxx Driver code

## GPIO driver (Done)

## SPI driver (In progress)

## I2C driver (Not yet)

## USART driver (Not yet)


## ETC

### Semihosting setup

- Project properties -> C/C++ build -> settings -> MCU GCC Linker -> Miscellaneous -> add 'Linker flags' `-specs=rdimon.specs -lc -lrdimon` -> Apply

- Build project -> check binary file `.elf` created -> Debug as -> Debug configuration -> Select binary at left menu -> Select *Startup* on right window -> At *Run Commands* add `monitor arm semihosting enable` -> Apply

- In `main.c` add this line below the includes `extern void initialise_monitor_handles();` add this code at the start of main function `initialise_monitor_handles();`

- `sysmem.c` - excluded from project

- Clean and Build

### Floating point exception 

- Properties -> C/C++ build -> settings -> MCU settings -> Floating point hardware to `No unit` -> Floating point ABI to `soft` -> Apply
