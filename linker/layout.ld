SECTIONS
{
    .shared_buffer (NOLOAD) : {
        . = ALIGN(4);
        *(.shared_buffer)
        . = ALIGN(4);
    } > SHARED_BUFFER
}

