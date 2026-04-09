set confirm off
set pagination off

target remote :3333
monitor arm semihosting disable
monitor cortex_m vector_catch none
monitor reset halt

thbreak main
continue

