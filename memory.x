MEMORY
{
    FLASH (rwx)        : ORIGIN = 0x08000000, LENGTH = 256K
    FLASH_SHARED (rwx) : ORIGIN = 0x08040000, LENGTH = 128K
    FLASH_TASK1 (rwx)  : ORIGIN = 0x08060000, LENGTH = 128K
    FLASH_TASK2 (rwx)  : ORIGIN = 0x08080000, LENGTH = 128K
    SHARED_BUFFER (rw): ORIGIN = 0x20000000, LENGTH = 512
    RAM (rwx)         : ORIGIN = 0x20000200, LENGTH = 640K - 512
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .shared_code :
    {
        . = ALIGN(4);
        KEEP(*(.shared_code .shared_code.*))
        . = ALIGN(4);
    } > FLASH_SHARED

    .tache1_section :
    {
        . = ALIGN(4);
        *(.tache1_text)
        *(.tache1_rodata)
        . = ALIGN(4);
    } > FLASH_TASK1

    .tache2_section :
    {
        . = ALIGN(4);
        *(.tache2_text)
        *(.tache2_rodata)
        . = ALIGN(4);
    } > FLASH_TASK2

    .shared_buffer (NOLOAD) :
    {
        . = ALIGN(4);
        *(.shared_buffer)
        . = ALIGN(4);
    } > SHARED_BUFFER

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
    } > RAM

    .data :
    {
        . = ALIGN(4);
        *(.data .data.*)
        *(COMMON)
        . = ALIGN(4);
    } > RAM
}

