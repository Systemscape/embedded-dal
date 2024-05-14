#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::I2C1;
use embassy_time::Duration;
use embedded_dal::config::{Dimensions, Orientation};
use embedded_dal::drivers::ft6x36::Ft6x36;
use embedded_dal::drivers::{dsi, ft6x36::TouchEvent, ltdc, nt35510::*};

use embassy_executor::Spawner;

use core::cell::RefCell;

use defmt::*;
use embassy_stm32::{
    dsihost::{blocking_delay_ms, DsiHost},
    gpio::{Level, Output, Pull, Speed},
    ltdc::Ltdc,
    pac::{
        dsihost::regs::{Ier0, Ier1},
        ltdc::vals::{Depol, Hspol, Pcpol, Pf, Vspol},
        DSIHOST, LTDC,
    },
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllRDiv, PllSource, Sysclk,
    },
    time::{mhz, Hertz},
};

use {defmt_rtt as _, panic_probe as _};

const LCD_ORIENTATION: Orientation = embedded_dal::config::Orientation::Landscape;
const LCD_DIMENSIONS: Dimensions = Dimensions::from(800, 480);

/* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
const LaneByteClk_kHz: u16 = 62500; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L224C21-L224C26

const LcdClock: u16 = 27429; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L183

/* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
const TXEscapeCkdiv: u8 = (LaneByteClk_kHz / 15620) as u8; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L230

const NUM_PIXELS: usize = LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as usize * LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as usize;

use slint::platform::software_renderer::{self, TargetPixel as _};

pub type TargetPixel = software_renderer::PremultipliedRgbaColor;

#[link_section = ".frame_buffer"]
static mut FB1: [TargetPixel; NUM_PIXELS] = [software_renderer::PremultipliedRgbaColor {
    red: 0,
    green: 0,
    blue: 0,
    alpha: 0,
}; NUM_PIXELS];

#[link_section = ".frame_buffer"]
static mut FB2: [TargetPixel; NUM_PIXELS] = [software_renderer::PremultipliedRgbaColor {
    red: 0,
    green: 0,
    blue: 0,
    alpha: 0,
}; NUM_PIXELS];

use embedded_alloc::Heap;

const HEAP_SIZE: usize = 50 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

struct StmBackendInner<'a> {
    scb: cortex_m::peripheral::SCB,
    delay: embassy_time::Delay,
    touch_screen: Ft6x36<I2c<'a, I2C1, Blocking>>,
}

struct StmBackend<'a> {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
    exti_input: ExtiInput<'a>,
    inner: RefCell<StmBackendInner<'a>>,
}

impl Default for StmBackend<'_> {
    fn default() -> Self {
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

        info!("Init Embassy...");

        let p = embassy_stm32::init(config);

        info!("Done!");

        let mut sdram = embassy_stm32::fmc::Fmc::sdram_a12bits_d32bits_4banks_bank1(
            p.FMC,
            p.PF0, // A0
            p.PF1,
            p.PF2,
            p.PF3,
            p.PF4,
            p.PF5, // A5
            p.PF12,
            p.PF13,
            p.PF14,
            p.PF15,
            p.PG0,
            p.PG1, // A11
            p.PG4, // BA0
            p.PG5,
            p.PD14, // D0
            p.PD15,
            p.PD0,
            p.PD1,
            p.PE7, // D4
            p.PE8,
            p.PE9,
            p.PE10,
            p.PE11,
            p.PE12,
            p.PE13,
            p.PE14,
            p.PE15, // D12
            p.PD8,
            p.PD9,
            p.PD10,
            p.PH8, // D16
            p.PH9,
            p.PH10,
            p.PH11,
            p.PH12,
            p.PH13,
            p.PH14,
            p.PH15, // D23
            p.PI0,
            p.PI1,
            p.PI2,
            p.PI3, // D27
            p.PI6, // D28
            p.PI7,
            p.PI9, // D30
            p.PI10,
            p.PE0, // NBL0
            p.PE1,
            p.PI4, // NBL2
            p.PI5,
            p.PH2,                                             // SDCKE0
            p.PG8,                                             // SDCLK
            p.PG15,                                            // SDNCAS
            p.PH3,                                             // SDNE0
            p.PF11,                                            // SDNRAS
            p.PC0,                                             // SDNWE
            stm32_fmc::devices::is42s32800g_6::Is42s32800g {}, // Not exactly the one on the Disco, but let's try...
        );

        let mut delay = embassy_time::Delay;

        let sdram_size = 32 * 256 * 4096;

        let ram_slice = unsafe {
            // Initialise controller and SDRAM
            let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

            info!("RAM ADDR: {:x}", ram_ptr as u32);

            // Convert raw pointer to slice
            core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u32>())
        };

        // // ----------------------------------------------------------
        // // Use memory in SDRAM
        info!("RAM contents before writing: {:x}", ram_slice[..10]);

        ram_slice[0] = 1;
        ram_slice[1] = 2;
        ram_slice[2] = 3;
        ram_slice[3] = 4;

        info!("RAM contents after writing: {:x}", ram_slice[..10]);

        crate::assert_eq!(ram_slice[0], 1);
        crate::assert_eq!(ram_slice[1], 2);
        crate::assert_eq!(ram_slice[2], 3);
        crate::assert_eq!(ram_slice[3], 4);

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
            embedded_dal::drivers::ft6x36::Dimension {
                x: LCD_DIMENSIONS.get_height(LCD_ORIENTATION),
                y: LCD_DIMENSIONS.get_width(LCD_ORIENTATION),
            },
        );

        touch_screen.init().unwrap();
        touch_screen.set_orientation(match LCD_ORIENTATION {
            Orientation::Landscape => embedded_dal::drivers::ft6x36::Orientation::Landscape,
            Orientation::Portrait => embedded_dal::drivers::ft6x36::Orientation::Portrait,
        });

        match touch_screen.get_info() {
            Some(info) => info!("Got touch screen info: {:#?}", info),
            None => warn!("No info"),
        }

        let diag = touch_screen.get_diagnostics().unwrap();
        info!("Got touch screen diag: {:#?}", diag);

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

        const HACT: u16 = LCD_DIMENSIONS.get_width(LCD_ORIENTATION);
        const VACT: u16 = LCD_DIMENSIONS.get_height(LCD_ORIENTATION);

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
        const PacketSize: u16 = HACT;
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
        DSIHOST
            .vhsacr()
            .modify(|w| w.set_hsa(dsi::to_clock_cycles(HSA, LaneByteClk_kHz, LcdClock)));

        /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
        DSIHOST
            .vhbpcr()
            .modify(|w| w.set_hbp(dsi::to_clock_cycles(HBP, LaneByteClk_kHz, LcdClock)));

        /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
        DSIHOST.vlcr().modify(|w| {
            w.set_hline(dsi::to_clock_cycles(
                HACT + HSA + HBP + HFP,
                //VACT + VSA + VBP + VFP,
                LaneByteClk_kHz,
                LcdClock,
            ))
        }); /* FIXME: Value depending on display orientation choice portrait/landscape */

        /* Set the Vertical Synchronization Active (VSA) */
        DSIHOST.vvsacr().modify(|w| w.set_vsa(VSA));

        /* Set the Vertical Back Porch (VBP)*/
        DSIHOST.vvbpcr().modify(|w| w.set_vbp(VBP));

        /* Set the Vertical Front Porch (VFP)*/
        DSIHOST.vvfpcr().modify(|w| w.set_vfp(VFP));

        /* Set the Vertical Active period*/
        DSIHOST.vvacr().modify(|w| w.set_va(VACT));

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
        const VerticalSync: u16 = VSA - 1;
        const AccumulatedHBP: u16 = HSA + HBP - 1;
        const AccumulatedVBP: u16 = VSA + VBP - 1;
        const AccumulatedActiveW: u16 = LCD_DIMENSIONS.get_width(LCD_ORIENTATION) + HSA + HBP - 1;
        const AccumulatedActiveH: u16 = LCD_DIMENSIONS.get_height(LCD_ORIENTATION) + VSA + VBP - 1;
        const TotalWidth: u16 = LCD_DIMENSIONS.get_width(LCD_ORIENTATION) + HSA + HBP + HFP - 1;
        const TotalHeight: u16 = LCD_DIMENSIONS.get_height(LCD_ORIENTATION) + VSA + VBP + VFP - 1;

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

        // FIXME: This should probably be done with a trait or so...
        let mut write_closure = |address: u8, data: &[u8]| dsi.write_cmd(0, address, data).unwrap();
        embedded_dal::drivers::nt35510::init(
            &mut write_closure,
            embassy_time::Delay,
            LCD_ORIENTATION,
        );

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

        ltdc::config_layer(
            ltdc::Window {
                x0: 200,
                x1: 260,
                y0: 300,
                y1: 360,
            },
            ltdc::Image {
                width: 60,
                height: 60,
            },
            Pf::ARGB8888,
            255,
            0,
            ltdc::RGB {
                red: 0,
                green: 0,
                blue: 0,
            },
            &FRAMEBUFFER[0] as *const _ as u32,
        );

        /*
        ######################################
         END HAL_LTDC_ConfigLayer()  and LTDC_SetConfig() for Layer 0
        ######################################
        */

        let mut cp = cortex_m::Peripherals::take().unwrap();

        Self {
            window: RefCell::default(),
            exti_input: embassy_stm32::exti::ExtiInput::new(p.PJ5, p.EXTI5, Pull::None),
            inner: RefCell::new(StmBackendInner {
                scb: cp.SCB,
                delay,
                touch_screen,
            }),
        }
    }
}

impl slint::platform::Platform for StmBackend<'_> {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let inner = &mut *self.inner.borrow_mut();

        // Safety: The Refcell at the beginning of `run_event_loop` prevents re-entrancy and thus multiple mutable references to FB1/FB2.
        let (fb1, fb2) = unsafe {
            (
                &mut *core::ptr::addr_of_mut!(FB1),
                &mut *core::ptr::addr_of_mut!(FB2),
            )
        };

        let mut displayed_fb: &mut [TargetPixel] = fb1;
        let mut work_fb: &mut [TargetPixel] = fb2;

        let mut last_touch = None;
        self.window
            .borrow()
            .as_ref()
            .unwrap()
            .set_size(slint::PhysicalSize::new(
                LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as u32,
                LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as u32,
            ));
        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    // Busy-wait for apply pending. Unnecessary?
                    //while ltdc::apply_pending() {}

                    renderer.render(work_fb, LCD_DIMENSIONS.get_width(LCD_ORIENTATION).into());
                    inner.scb.clean_dcache_by_slice(work_fb); // Unsure... DCache and Ethernet may cause issues.

                    ltdc::set_framebuffer(work_fb.as_ptr() as *const u32);
                    core::mem::swap::<&mut [_]>(&mut work_fb, &mut displayed_fb);
                    info!("Swapped FrameBuffer: {:#x}", unsafe { *(work_fb.as_ptr() as *const u32) });

                    /*

                    // FIXME: Here we need to interface with the LTDC and take care of swapping the frame buffer. Maybe checkout what is_swap_pending and swap_framebuffer of the other HAL do.

                    while inner.layer.is_swap_pending() {} // We do this with apply config. Maybe rework this...
                    renderer.render(work_fb, LCD_DIMENSIONS.get_width(LCD_ORIENTATION).into());
                    inner.scb.clean_dcache_by_slice(work_fb);
                    // Safety: the frame buffer has the right size
                    unsafe { inner.layer.swap_framebuffer(work_fb.as_ptr() as *const u8) };
                    // Swap the buffer pointer so we will work now on the second buffer
                    core::mem::swap::<&mut [_]>(&mut work_fb, &mut displayed_fb);

                    */
                });

                // handle touch event
                let button = slint::platform::PointerEventButton::Left;
                let event = match inner.touch_screen.get_touch_event().map(|x| x.p1).ok().flatten() {
                    Some(state) => {
                        let position = slint::PhysicalPosition::new(state.y as i32, state.x as i32)
                            .to_logical(window.scale_factor());

                        info!("Got Touch!");
                        Some(match last_touch.replace(position) {
                            Some(_) => slint::platform::WindowEvent::PointerMoved { position },
                            None => {
                                slint::platform::WindowEvent::PointerPressed { position, button }
                            }
                        })
                    }
                    None => last_touch.take().map(|position| {
                        slint::platform::WindowEvent::PointerReleased { position, button }
                    }),
                };

                if let Some(event) = event {
                    let is_pointer_release_event =
                        matches!(event, slint::platform::WindowEvent::PointerReleased { .. });

                    window.dispatch_event(event);

                    // removes hover state on widgets
                    if is_pointer_release_event {
                        window.dispatch_event(slint::platform::WindowEvent::PointerExited);
                    }
                }
            }

            // FIXME: cortex_m::asm::wfe();
        }
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(embassy_time::Instant::now().as_micros())
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    /*
    loop {
        let time =
            embassy_time::Instant::now().duration_since(embassy_time::Instant::from_ticks(0));
        self.exti_pin.wait_for_low().await;
        let event = touch_screen.get_touch_event().unwrap();
        //.ok()
        //.and_then(|touch_event| touch_screen.process_event(time.into(), touch_event));
        if let Some(point) = event.p1 {
            info!("Got event: {:#?}", event);

            ltdc::config_window(
                ltdc::Window {
                    x0: point.x,
                    x1: point.x + 60,
                    y0: point.y,
                    y1: point.y + 60,
                },
                Pf::ARGB8888,
            );
        }


        //embassy_time::block_for(embassy_time::Duration::from_millis(10));


        if let Some(_) = event.p1 {
            embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0xFF);
        } else {
            embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, 0x20);
        }
        embassy_time::block_for(Duration::from_millis(100));

    }
    */

    /*
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
    */

    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
    slint::platform::set_platform(Box::new(StmBackend::default()))
        .expect("backend already initialized");

    let window = MainWindow::new().unwrap();
    window.run().unwrap();
}

slint::include_modules!();
