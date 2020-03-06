OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x0, LENGTH = 4K
}

STACK_SIZE  = 512;
HEAP_SIZE   = 64;

SECTIONS
{
    .text : ALIGN(4)
    {
        __start_addr = .;
        KEEP(*(.text.entry))
        . = ALIGN(4);
        *(.text*)
        . = ALIGN(4);
       __text_end = .;
    } > RAM

    .rodata : ALIGN(4)
    {
        __rodata_start = .;
        *(.rodata*)
        *(.srodata*)
        . = ALIGN(4);
        __rodata_end = .;
        __load_start = .;
    } > RAM

    .data : AT(ADDR(.text)+SIZEOF(.text)+SIZEOF(.rodata))
    {
        __data_start = .;
        *(.data*)
        *(.sdata*)
        . = ALIGN(4);
        __data_end = .;
        _gp = . + 0x100;
    } > RAM

    .bss : ALIGN(4)
    {
        __bss_begin = .;
        *(.sbss*)
        *(.scommon)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM
    _end = .;

    .stack : ALIGN(4)
    {
        __stack_bottom = .;
        . += STACK_SIZE;
        __stack_top = .;
    } > RAM

    .heap : ALIGN(4)
    {
        __heap_begin = .;
        . += HEAP_SIZE;
        __heap_end = .;
    } > RAM
}

