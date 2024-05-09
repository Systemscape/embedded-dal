#![no_std]
#![no_main]

use embassy_time::Duration;
use embedded_dal::drivers::nt35510::*;

use embassy_executor::Spawner;

use defmt::*;
use embassy_stm32::{
    dsihost::{blocking_delay_ms, DsiHost, PacketType},
    gpio::{Input, Level, Output, Pull, Speed},
    ltdc::Ltdc,
    pac::{
        dsihost::regs::{Ier0, Ier1},
        ltdc::vals::{Bf1, Bf2, Depol, Hspol, Imr, Pcpol, Pf, Vspol},
        DSIHOST, LTDC,
    },
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllRDiv, PllSource, Sysclk,
    },
    time::{mhz, Hertz},
};

use {defmt_rtt as _, panic_probe as _};

enum Orientation {
    Landscape,
    Portrait,
}

const LCD_Orientation: Orientation = Orientation::Landscape;
const LCD_X_Size: u16 = 800;
const LCD_Y_Size: u16 = 480;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    /*
        SystemClock_Config()
    */

    let mut config = embassy_stm32::Config::default();
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;

    // HSE is on and ready
    config.rcc.hse = Some(Hse {
        freq: mhz(8),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;

    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8, // PLLM
        mul: PllMul::MUL360,     // PLLN
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7), // was DIV4, but STM BSP example uses 7
        divr: Some(PllRDiv::DIV6),
    });

    // This seems to be working, the values in the RCC.PLLSAICFGR are correct according to the debugger. Also on and ready according to CR
    config.rcc.pllsai = Some(Pll {
        prediv: PllPreDiv::DIV8,   // Actually ignored
        mul: PllMul::MUL384,       // PLLN
        divp: None,                // PLLP
        divq: None,                // PLLQ
        divr: Some(PllRDiv::DIV7), // PLLR (Sai actually has special clockdiv register)
    });

    let p = embassy_stm32::init(config);
    info!("Starting...");

    /*
       BSP_LCD_Reset() !!! ALSO RESETS TOUCHSCREEN - Necessary before using TS !!!
    */

    // According to UM for the discovery kit, PH7 is an active-low reset for the LCD and touchsensor
    let mut reset = Output::new(p.PH7, Level::Low, Speed::High);

    // CubeMX example waits 20 ms before de-asserting reset. Must be at least 5ms for the touch screen.
    embassy_time::block_for(embassy_time::Duration::from_millis(20));

    // Disable the reset signal and wait 140ms as in the Linux driver (CubeMX waits only 20)
    reset.set_high();
    embassy_time::block_for(embassy_time::Duration::from_millis(140));

    // BEGIN TOUCHSCREEN

    // "Time of starting to report point after resetting" according to TS datasheet is 300 ms after reset
    //embassy_time::block_for(embassy_time::Duration::from_millis(160));

    let mut i2c = embassy_stm32::i2c::I2c::new_blocking(
        p.I2C1,
        p.PB8,
        p.PB9,
        Hertz(400_000),
        Default::default(),
    );

    let mut touch_screen = embedded_dal::drivers::ft6x36::Ft6x36::new(
        i2c,
        0x38,
        embedded_dal::drivers::ft6x36::Dimension(LCD_Y_Size, LCD_X_Size),
    );

    touch_screen.init().unwrap();

    match touch_screen.get_info() {
        Some(info) => info!("Got touch screen info: {:#?}", info),
        None => warn!("No info"),
    }

    // END TOUCHSCREEN

    let mut led = Output::new(p.PG6, Level::High, Speed::Low);

    /*
    BSP_LCD_MspInit() // This will set IP blocks LTDC, DSI and DMA2D => We should be fine with what Embassy does.
     */

    let mut ltdc = Ltdc::new(p.LTDC);
    let mut dsi = DsiHost::new(p.DSIHOST, p.PJ2);
    let version = dsi.get_version();
    defmt::warn!("DSI IP Version: {:x}", version);

    /*
        HAL_DSI_DeInit()
    */

    // Disable the DSI wrapper
    dsi.disable_wrapper_dsi();

    // Disable the DSI host
    dsi.disable();

    // D-PHY clock and digital disable
    DSIHOST.pctlr().modify(|w| {
        w.set_cke(false);
        w.set_den(false)
    });

    // Turn off the DSI PLL
    DSIHOST.wrpcr().modify(|w| w.set_pllen(false));

    // Disable the regulator
    DSIHOST.wrpcr().write(|w| w.set_regen(false));

    /*
    ######################################
    BEGIN HAL_DSI_Init()
    ######################################
    */

    // Enable regulator (__HAL_DSI_REG_ENABLE)
    info!("DSIHOST: enabling regulator");
    DSIHOST.wrpcr().write(|w| w.set_regen(true));

    for _ in 1..1000 {
        // The regulator status (ready or not) can be monitored with the RRS flag in the DSI_WISR register.
        // Once it is set, we stop waiting.
        if DSIHOST.wisr().read().rrs() {
            info!("DSIHOST Regulator ready");
            break;
        }
        embassy_time::block_for(embassy_time::Duration::from_millis(1));
    }

    if !DSIHOST.wisr().read().rrs() {
        defmt::panic!("DSIHOST: enabling regulator FAILED");
    }

    // Set up PLL and enable it
    DSIHOST.wrpcr().modify(|w| {
        w.set_pllen(true);
        w.set_ndiv(125); // PLL loop division factor set to 125
        w.set_idf(2); // PLL input divided by 2
        w.set_odf(0); // PLL output divided by 1
    });

    /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
    const LaneByteClk_kHz: u16 = 62500; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L224C21-L224C26

    const LcdClock: u16 = 27429; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L183

    /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
    const TXEscapeCkdiv: u8 = (LaneByteClk_kHz / 15620) as u8; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L230

    for _ in 1..1000 {
        embassy_time::block_for(embassy_time::Duration::from_millis(1));
        // The PLL status (lock or unlock) can be monitored with the PLLLS flag in the DSI_WISR register.
        // Once it is set, we stop waiting.
        if DSIHOST.wisr().read().pllls() {
            info!("DSIHOST PLL locked");
            break;
        }
    }

    if !DSIHOST.wisr().read().pllls() {
        defmt::panic!("DSIHOST: enabling PLL FAILED");
    }

    // ### Set the PHY parameters ###

    // D-PHY clock and digital enable
    DSIHOST.pctlr().write(|w| {
        w.set_cke(true);
        w.set_den(true);
    });

    // Set Clock lane to high-speed mode and disable automatic clock lane control
    DSIHOST.clcr().modify(|w| {
        w.set_dpcc(true);
        w.set_acr(false); // TODO: Verify that false is correct here. Seems unset in BSP, i.e., zero/false.
    });

    // Set number of active data lanes to two (lanes 0 and 1)
    DSIHOST.pconfr().modify(|w| w.set_nl(1));

    // ### Set the DSI clock parameters ###

    // Set the TX escape clock division factor to 4
    DSIHOST.ccr().modify(|w| w.set_txeckdiv(TXEscapeCkdiv));

    // Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4)
    // The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )
    // Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)
    // Set the bit period in high-speed mode
    DSIHOST.wpcr0().modify(|w| w.set_uix4(8)); // 8 is set in the BSP example (confirmed with Debugger)

    // Disable all error interrupts and reset the Error Mask
    DSIHOST.ier0().write_value(Ier0(0));
    DSIHOST.ier1().write_value(Ier1(0));

    // Enable this to fix read timeout
    DSIHOST.pcr().modify(|w| w.set_btae(true));

    /*
    ######################################
    END HAL_DSI_Init()
    ######################################
    */

    const DSI_PIXELFORMAT_RGB888: u8 = 0x05;
    const DSI_PIXELFORMAT_ARGB888: u8 = 0x00;

    const HACT: u16 = LCD_X_Size;
    const VACT: u16 = LCD_Y_Size;

    const VSA: u16 = 120;
    const VBP: u16 = 150;
    const VFP: u16 = 150;
    const HSA: u16 = 2;
    const HBP: u16 = 34;
    const HFP: u16 = 34;

    const VirtualChannelID: u8 = 0;

    const ColorCoding: u8 = DSI_PIXELFORMAT_RGB888;
    const VSPolarity: bool = false; // DSI_VSYNC_ACTIVE_HIGH == 0
    const HSPolarity: bool = false; // DSI_HSYNC_ACTIVE_HIGH == 0
    const DEPolarity: bool = false; // DSI_DATA_ENABLE_ACTIVE_HIGH == 0
    const Mode: u8 = 2; // DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
    const NullPacketSize: u16 = 0xFFF;
    const NumberOfChunks: u16 = 0;
    const PacketSize: u16 = HACT; /* Value depending on display orientation choice portrait/landscape */
    const HorizontalSyncActive: u16 = 4; // ((HSA as u32 * LaneByteClk_kHz as u32 ) / LcdClock as u32 ) as u16;
    const HorizontalBackPorch: u16 = 77; //((HBP as u32  * LaneByteClk_kHz as u32 ) / LcdClock as u32) as u16;
    const HorizontalLine: u16 = 1982; //(((HACT + HSA + HBP + HFP) as u32  * LaneByteClk_kHz as u32 ) / LcdClock as u32 ) as u16; /* Value depending on display orientation choice portrait/landscape */
                                      // FIXME: Make depend on orientation
    const VerticalSyncActive: u16 = VSA;
    const VerticalBackPorch: u16 = VBP;
    const VerticalFrontPorch: u16 = VFP;
    const VerticalActive: u16 = VACT;
    const LPCommandEnable: bool = true; /* Enable sending commands in mode LP (Low Power) */

    /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
    /* Only useful when sending LP packets is allowed while streaming is active in video mode */
    const LPLargestPacketSize: u8 = 16;

    /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
    /* Only useful when sending LP packets is allowed while streaming is active in video mode */
    const LPVACTLargestPacketSize: u8 = 0;

    const LPHorizontalFrontPorchEnable: bool = true; /* Allow sending LP commands during HFP period */
    const LPHorizontalBackPorchEnable: bool = true; /* Allow sending LP commands during HBP period */
    const LPVerticalActiveEnable: bool = true; /* Allow sending LP commands during VACT period */
    const LPVerticalFrontPorchEnable: bool = true; /* Allow sending LP commands during VFP period */
    const LPVerticalBackPorchEnable: bool = true; /* Allow sending LP commands during VBP period */
    const LPVerticalSyncActiveEnable: bool = true; /* Allow sending LP commands during VSync = VSA period */
    const FrameBTAAcknowledgeEnable: bool = false; /* Frame bus-turn-around acknowledge enable => false according to debugger */

    /*
    ######################################
    BEGIN HAL_DSI_ConfigVideoMode()
    ######################################
    */

    /*
        if (VidCfg->ColorCoding == DSI_RGB666)
    {
      assert_param(IS_DSI_LOOSELY_PACKED(VidCfg->LooselyPacked));
    }
    */

    /* Select video mode by resetting CMDM and DSIM bits */
    DSIHOST.mcr().modify(|w| w.set_cmdm(false));
    DSIHOST.wcfgr().modify(|w| w.set_dsim(false));

    /* Configure the video mode transmission type */
    DSIHOST.vmcr().modify(|w| w.set_vmt(Mode));

    /* Configure the video packet size */
    DSIHOST.vpcr().modify(|w| w.set_vpsize(PacketSize));

    /* Set the chunks number to be transmitted through the DSI link */
    DSIHOST.vccr().modify(|w| w.set_numc(NumberOfChunks));

    /* Set the size of the null packet */
    DSIHOST.vnpcr().modify(|w| w.set_npsize(NullPacketSize));

    /* Select the virtual channel for the LTDC interface traffic */
    DSIHOST.lvcidr().modify(|w| w.set_vcid(VirtualChannelID));

    /* Configure the polarity of control signals */
    DSIHOST.lpcr().modify(|w| {
        w.set_dep(DEPolarity);
        w.set_hsp(HSPolarity);
        w.set_vsp(VSPolarity);
    });

    /* Select the color coding for the host */
    DSIHOST.lcolcr().modify(|w| w.set_colc(ColorCoding));

    /* Select the color coding for the wrapper */
    DSIHOST.wcfgr().modify(|w| w.set_colmux(ColorCoding));

    /* // FIXME
    /* Enable/disable the loosely packed variant to 18-bit configuration */
    if (VidCfg->ColorCoding == DSI_RGB666)
    {
      hdsi->Instance->LCOLCR &= ~DSI_LCOLCR_LPE;
      hdsi->Instance->LCOLCR |= VidCfg->LooselyPacked;
    }
    */

    /* Set the Horizontal Synchronization Active (HSA) in lane byte clock cycles */
    DSIHOST.vhsacr().modify(|w| w.set_hsa(HorizontalSyncActive));

    /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
    DSIHOST.vhbpcr().modify(|w| w.set_hbp(HorizontalBackPorch));

    /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
    DSIHOST.vlcr().modify(|w| w.set_hline(HorizontalLine));

    /* Set the Vertical Synchronization Active (VSA) */
    DSIHOST.vvsacr().modify(|w| w.set_vsa(VerticalSyncActive));

    /* Set the Vertical Back Porch (VBP)*/
    DSIHOST.vvbpcr().modify(|w| w.set_vbp(VerticalBackPorch));

    /* Set the Vertical Front Porch (VFP)*/
    DSIHOST.vvfpcr().modify(|w| w.set_vfp(VerticalFrontPorch));

    /* Set the Vertical Active period*/
    DSIHOST.vvacr().modify(|w| w.set_va(VerticalActive));

    /* Configure the command transmission mode */
    DSIHOST.vmcr().modify(|w| w.set_lpce(LPCommandEnable));

    /* Low power largest packet size */
    DSIHOST
        .lpmcr()
        .modify(|w| w.set_lpsize(LPLargestPacketSize));

    /* Low power VACT largest packet size */
    DSIHOST
        .lpmcr()
        .modify(|w| w.set_lpsize(LPLargestPacketSize));
    DSIHOST
        .lpmcr()
        .modify(|w| w.set_vlpsize(LPVACTLargestPacketSize));

    /* Enable LP transition in HFP period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lphfpe(LPHorizontalFrontPorchEnable));

    /* Enable LP transition in HBP period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lphbpe(LPHorizontalBackPorchEnable));

    /* Enable LP transition in VACT period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lpvae(LPVerticalActiveEnable));

    /* Enable LP transition in VFP period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lpvfpe(LPVerticalFrontPorchEnable));

    /* Enable LP transition in VBP period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lpvbpe(LPVerticalBackPorchEnable));

    /* Enable LP transition in vertical sync period */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_lpvsae(LPVerticalSyncActiveEnable));

    /* Enable the request for an acknowledge response at the end of a frame */
    DSIHOST
        .vmcr()
        .modify(|w| w.set_fbtaae(FrameBTAAcknowledgeEnable));

    /*
    ######################################
    END HAL_DSI_ConfigVideoMode()
    ######################################
    */

    /* Configure DSI PHY HS2LP and LP2HS timings */
    const ClockLaneHS2LPTime: u16 = 35;
    const ClockLaneLP2HSTime: u16 = 35;
    const DataLaneHS2LPTime: u8 = 35;
    const DataLaneLP2HSTime: u8 = 35;
    const DataLaneMaxReadTime: u16 = 0;
    const StopWaitTime: u8 = 10;

    const MAX_TIME: u16 = if ClockLaneHS2LPTime > ClockLaneLP2HSTime {
        ClockLaneHS2LPTime
    } else {
        ClockLaneLP2HSTime
    };

    /*
    ######################################
     BEGIN HAL_DSI_ConfigPhyTimer()
    ######################################
    */

    /* Clock lane timer configuration */

    /* In Automatic Clock Lane control mode, the DSI Host can turn off the clock lane between two
     High-Speed transmission.
     To do so, the DSI Host calculates the time required for the clock lane to change from HighSpeed
     to Low-Power and from Low-Power to High-Speed.
     This timings are configured by the HS2LP_TIME and LP2HS_TIME in the DSI Host Clock Lane Timer Configuration
     Register (DSI_CLTCR).
     But the DSI Host is not calculating LP2HS_TIME + HS2LP_TIME but 2 x HS2LP_TIME.

     Workaround : Configure HS2LP_TIME and LP2HS_TIME with the same value being the max of HS2LP_TIME or LP2HS_TIME.
    */

    DSIHOST.cltcr().modify(|w| {
        w.set_hs2lp_time(MAX_TIME);
        w.set_lp2hs_time(MAX_TIME)
    });

    // Data lane timer configuration
    DSIHOST.dltcr().modify(|w| {
        w.set_hs2lp_time(DataLaneHS2LPTime);
        w.set_lp2hs_time(DataLaneLP2HSTime);
        w.set_mrd_time(DataLaneMaxReadTime);
    });

    // Configure the wait period to request HS transmission after a stop state
    DSIHOST.pconfr().modify(|w| w.set_sw_time(StopWaitTime));

    /*
    ######################################
     END HAL_DSI_ConfigPhyTimer()
    ######################################
    */

    // Here comes HAL_DSI_ConfigPhyTimer() in the BSP example. We have already configured it at the beginning.

    const PCPolarity: bool = false; // LTDC_PCPOLARITY_IPC == 0

    const LTDC_DEPolarity: Depol = if DEPolarity == false {
        Depol::ACTIVELOW
    } else {
        Depol::ACTIVEHIGH
    };
    const LTDC_VSPolarity: Vspol = if VSPolarity == false {
        Vspol::ACTIVEHIGH
    } else {
        Vspol::ACTIVELOW
    };

    const LTDC_HSPolarity: Hspol = if HSPolarity == false {
        Hspol::ACTIVEHIGH
    } else {
        Hspol::ACTIVELOW
    };

    /* Timing Configuration */
    const HorizontalSync: u16 = HSA - 1;
    const VerticalSync: u16 = VerticalSyncActive - 1;
    const AccumulatedHBP: u16 = HSA + HBP - 1;
    const AccumulatedVBP: u16 = VerticalSyncActive + VerticalBackPorch - 1;
    const AccumulatedActiveW: u16 = LCD_X_Size + HSA + HBP - 1;
    const AccumulatedActiveH: u16 = VerticalSyncActive + VerticalBackPorch + VerticalActive - 1;
    const TotalWidth: u16 = LCD_X_Size + HSA + HBP + HFP - 1;
    const TotalHeight: u16 =
        VerticalSyncActive + VerticalBackPorch + VerticalActive + VerticalFrontPorch - 1;

    /*
    ######################################
     BEGIN HAL_LTDC_Init()
    ######################################
    */

    // DISABLE LTDC before making changes
    ltdc.disable();

    // Configure the HS, VS, DE and PC polarity
    LTDC.gcr().modify(|w| {
        w.set_hspol(LTDC_HSPolarity);
        w.set_vspol(LTDC_VSPolarity);
        w.set_depol(LTDC_DEPolarity);
        w.set_pcpol(Pcpol::RISINGEDGE); // #define LTDC_PCPOLARITY_IPC 0x00000000U
    });

    // Set Synchronization size
    LTDC.sscr().modify(|w| {
        w.set_hsw(HorizontalSync);
        w.set_vsh(VerticalSync)
    });

    // Set Accumulated Back porch
    LTDC.bpcr().modify(|w| {
        w.set_ahbp(AccumulatedHBP);
        w.set_avbp(AccumulatedVBP);
    });

    // Set Accumulated Active Width
    LTDC.awcr().modify(|w| {
        w.set_aah(AccumulatedActiveH);
        w.set_aaw(AccumulatedActiveW);
    });

    // Set Total Width
    LTDC.twcr().modify(|w| {
        w.set_totalh(TotalHeight);
        w.set_totalw(TotalWidth);
    });

    // Set the background color value
    LTDC.bccr().modify(|w| {
        w.set_bcred(0);
        w.set_bcgreen(0);
        w.set_bcblue(0)
    });

    // Enable the Transfer Error and FIFO underrun interrupts
    LTDC.ier().modify(|w| {
        w.set_terrie(true);
        w.set_fuie(true);
    });

    // ENABLE LTDC after making changes
    ltdc.enable();

    /*
    ######################################
     END HAL_LTDC_Init()
    ######################################
    */

    /*
    ######################################
     BEGIN HAL_DSI_Start()
    ######################################
    */

    dsi.enable();
    dsi.enable_wrapper_dsi();

    /*
    ######################################
     END HAL_DSI_Start()
    ######################################
    */

    /*
    ######################################
     BEGIN NT35510_Init(()
    ######################################
    */

    let mut write_closure = |address: u8, data: &[u8]| dsi.write_cmd(0, address, data).unwrap();
    embedded_dal::drivers::nt35510::init(&mut write_closure, embassy_time::Delay);

    /*
    ######################################
     END NT35510_Init(()
    ######################################
    */

    use embedded_dal::test_images::systemscape_logo::FRAMEBUFFER;


    /* Initialize the LCD pixel width and pixel height */
    const WindowX0: u16 = 200; //100;
    const WindowX1: u16 = 260; //100 + 60; // // 60 for camera
    const WindowY0: u16 = 200; //100;
    const WindowY1: u16 = 260; //100 + 60; //LCD_Y_Size; // 60 for camera
    const PixelFormat: Pf = Pf::ARGB8888;
    const Alpha: u8 = 255;
    const Alpha0: u8 = 0;
    const BackcolorBlue: u8 = 0;
    const BackcolorGreen: u8 = 0;
    const BackcolorRed: u8 = 0;
    const ImageWidth: u16 = 60; //LCD_X_Size; // 60 for camera
    const ImageHeight: u16 = 60; //LCD_Y_Size; // 60 for camera


    /*
    use embedded_dal::test_images::systemscape::FRAMEBUFFER;

    /* Initialize the LCD pixel width and pixel height */
    const WindowX0: u16 = 0; //100;
    const WindowX1: u16 = LCD_X_Size; //100 + 60; // // 60 for camera
    const WindowY0: u16 = 0; //100;
    const WindowY1: u16 = LCD_Y_Size; //100 + 60; //LCD_Y_Size; // 60 for camera
    const PixelFormat: Pf = Pf::ARGB8888;
    const Alpha: u8 = 255;
    const Alpha0: u8 = 0;
    const BackcolorBlue: u8 = 0;
    const BackcolorGreen: u8 = 0;
    const BackcolorRed: u8 = 0;
    const ImageWidth: u16 = LCD_X_Size; //60; //LCD_X_Size; // 60 for camera
    const ImageHeight: u16 = LCD_Y_Size; //60; //LCD_Y_Size; // 60 for camera
    */

    /*:
    ######################################
     BEGIN HAL_LTDC_ConfigLayer() and LTDC_SetConfig() for Layer 0
    ######################################
    */

    const PIXEL_SIZE: u8 = match PixelFormat {
        Pf::ARGB8888 => 4,
        Pf::RGB888 => 3,
        Pf::ARGB4444 | Pf::RGB565 | Pf::ARGB1555 | Pf::AL88 => 2,
        _ => 1,
    };

    // Configure the horizontal start and stop position
    LTDC.layer(0).whpcr().write(|w| {
        w.set_whstpos(LTDC.bpcr().read().ahbp() + 1 + WindowX0);
        w.set_whsppos(LTDC.bpcr().read().ahbp() + WindowX1);
    });

    // Configures the vertical start and stop position
    LTDC.layer(0).wvpcr().write(|w| {
        w.set_wvstpos(LTDC.bpcr().read().avbp() + 1 + WindowY0);
        w.set_wvsppos(LTDC.bpcr().read().avbp() + WindowY1);
    });

    // Specify the pixel format
    LTDC.layer(0).pfcr().write(|w| w.set_pf(PixelFormat));

    // Configures the default color values as zero
    LTDC.layer(0).dccr().modify(|w| {
        w.set_dcblue(BackcolorBlue);
        w.set_dcgreen(BackcolorGreen);
        w.set_dcred(BackcolorRed);
        w.set_dcalpha(Alpha0);
    });

    // Specifies the constant alpha value
    LTDC.layer(0).cacr().write(|w| w.set_consta(Alpha));

    // Specifies the blending factors
    LTDC.layer(0).bfcr().write(|w| {
        w.set_bf1(Bf1::CONSTANT);
        w.set_bf2(Bf2::CONSTANT);
    });

    // Configure the color frame buffer start address
    let fb_start_address: u32 = &FRAMEBUFFER[0] as *const _ as u32; // TODO: REPLACE WITH REAL ADDRESS
    info!(
        "Setting Framebuffer Start Address: {:010x}",
        fb_start_address
    );
    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(fb_start_address));

    // Configures the color frame buffer pitch in byte
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(ImageWidth * PIXEL_SIZE as u16);
        w.set_cfbll(((WindowX1 - WindowX0) * PIXEL_SIZE as u16) + 3);
    });

    // Configures the frame buffer line number
    LTDC.layer(0).cfblnr().write(|w| w.set_cfblnbr(ImageHeight));

    // Enable LTDC_Layer by setting LEN bit
    LTDC.layer(0).cr().modify(|w| w.set_len(true));

    // Comes after LTDC_SetConfig() in HAL_LTDC_ConfigLayer()
    //LTDC->SRCR = LTDC_SRCR_IMR;
    LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

    /*
    ######################################
     END HAL_LTDC_ConfigLayer()  and LTDC_SetConfig() for Layer 0
    ######################################
    */

    blocking_delay_ms(500);

    /*/

    const READ_SIZE: u16 = 1;
    let mut data = [1u8; READ_SIZE as usize];
    dsi.read(0, PacketType::DcsShortPktRead(0xDA), READ_SIZE, &mut data)
        .unwrap();
    info!("Display ID1: {:#04x}", data);

    dsi.read(0, PacketType::DcsShortPktRead(0xDB), READ_SIZE, &mut data)
        .unwrap();
    info!("Display ID2: {:#04x}", data);

    dsi.read(0, PacketType::DcsShortPktRead(0xDC), READ_SIZE, &mut data)
        .unwrap();
    info!("Display ID3: {:#04x}", data);

    blocking_delay_ms(500);

    */

        //let mut ts_int = Input::new(p.PJ5, Pull::None);
    let mut exti_pin = embassy_stm32::exti::ExtiInput::new(p.PJ5, p.EXTI5, Pull::None);
    loop {
        let time = embassy_time::Instant::now().duration_since(embassy_time::Instant::from_ticks(0));
        exti_pin.wait_for_falling_edge().await;
        let event = touch_screen
            .get_touch_event()
            .ok()
            .and_then(|touch_event| touch_screen.process_event(time.into(), touch_event));
        if let Some(event) = event {
            info!("Got event: {:#?}", event);
        } 
        //embassy_time::block_for(embassy_time::Duration::from_millis(10));

        /*
        if let Some(_) = event.p1 {
            embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0xFF);
        } else {
            embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0x20);
        }
        embassy_time::block_for(Duration::from_millis(100));
        */
    }

    info!("Config done, start blinking LED");
    loop {
        led.set_high();
        embassy_time::block_for(embassy_time::Duration::from_millis(2000));

        // Increase screen brightness
        embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0xFF);
        //dsi.write_cmd(0, NT35510_CMD_WRDISBV, &[0xFF]).unwrap();

        //dsi.write_cmd(0, &[NT35510_CMD_ALLPIXELS_ON, 0xFF]);

        led.set_low();
        embassy_time::block_for(embassy_time::Duration::from_millis(2000));

        // Reduce screen brightness
        //dsi.write_cmd(0, NT35510_CMD_WRDISBV, &[0x50]).unwrap();
        embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0x30);

        //dsi.write_cmd(0, &[NT35510_CMD_ALLPIXELS_OFF, 0xFF]);
    }
}
