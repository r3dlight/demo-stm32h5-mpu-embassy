SECTIONS
{
    .vector_table :
    {
        KEEP(*(.vector_table .vector_table.*))
        . = ALIGN(256);
    } > FLASH AT> FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > FLASH

    .tache1_section :
    {
        *(.tache1_text)
        *(.tache1_rodata)
    } > FLASH_TASK_1

    .tache2_section :
    {
        *(.tache2_text)
        *(.tache2_rodata)
    } > FLASH_TASK_2

    .shared_buffer (NOLOAD) : {
        . = ALIGN(4);
        *(.shared_buffer)
        . = ALIGN(4);
    } > SHARED_BUFFER
}

