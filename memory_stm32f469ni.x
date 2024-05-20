MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 2048K /* BANK_1_REGION_1 + BANK_1_REGION_2 + BANK_1_REGION_3 + BANK_2_REGION_1 + BANK_2_REGION_2 + BANK_2_REGION_3 */
    RAM   : ORIGIN = 0x20000000, LENGTH =  320K
    OTP   : ORIGIN = 0x1fff7800, LENGTH =  528
    QSPI  : ORIGIN = 0x90000000, LENGTH = 16M
    SDRAM : ORIGIN = 0xC0000000, LENGTH = 16M
}


_stack_start = ORIGIN(RAM) + LENGTH(RAM);


SECTIONS {
     .frame_buffer (NOLOAD) : {
       . = ALIGN(4);
       *(.frame_buffer);
       . = ALIGN(4);
     } > SDRAM

    .qspi_flash : {
       . = ALIGN(4);
       *(.qspi_flash);
       . = ALIGN(4);
     } > QSPI

/*
     .slint_assets : {
       . = ALIGN(4);
      __s_slint_assets = .;
       *(.slint_assets);
       . = ALIGN(4);
     } > SDRAM  AT>OSPI_ROM

    __e_slint_assets = .;
    __si_slint_assets = LOADADDR(.slint_assets);
    */
}

