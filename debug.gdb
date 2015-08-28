target remote :2331
monitor halt
file binaries/at91bootstrap.elf
load
break main
break load_dataflash