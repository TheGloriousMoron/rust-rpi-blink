ENTRY(_start)

MEMORY
{
    ROM (rx)  : ORIGIN = 0x00008000, LENGTH = 1M  /* Change as needed */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 1M  /* Change as needed */
}

SECTIONS
{
    .text :
    {
        *(.text)     /* Code */
    } > ROM

    .data :
    {
        *(.data)     /* Initialized data */
    } > RAM

    .bss :
    {
        *(.bss)      /* Uninitialized data */
    } > RAM

    .stack :
    {
        *(.stack)    /* Stack section */
    } > RAM

    .heap :
    {
        *(.heap)     /* Heap section */
    } > RAM
}
