/* STM32H723VG：1 MiB 闪存，以及任一 RAM 分区均可使用的 128 KiB AXI SRAM。 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   : ORIGIN = 0x24000000, LENGTH = 128K
}

_stext = ALIGN(ADDR(.vector_table) + SIZEOF(.vector_table), 8);
