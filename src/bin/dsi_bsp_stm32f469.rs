#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;

use cortex_m::peripheral::SCB;

use embassy_time::Duration;
use embedded_dal::{
    config::{Dimensions, Orientation},
    drivers::{
        dsi::{self, Dsi},
        ft6x36::{Ft6x36, TouchEvent},
        ltdc,
        nt35510::*,
    },
};

use embassy_executor::Spawner;

use core::cell::RefCell;

use cortex_mpu::Size;
use defmt::*;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Speed},
    i2c::I2c,
    ltdc::Ltdc,
    mode::Blocking,
    pac::{
        ltdc::vals::{Depol, Hspol, Pcpol, Pf, Vspol},
        LTDC,
    },
    peripherals::{DSIHOST, EXTI5, I2C1, LTDC, PB8, PB9, PG6, PH7, PJ2, PJ5},
    rcc::{Hse, HseMode, Pll},
    time::{mhz, Hertz},
};
use embassy_stm32::{
    gpio::Pull,
    rcc::{
        AHBPrescaler, APBPrescaler, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource, Sysclk,
    },
};

use {defmt_rtt as _, panic_probe as _};

const LCD_ORIENTATION: Orientation = embedded_dal::config::Orientation::Landscape;
const LCD_DIMENSIONS: Dimensions = Dimensions::from(800, 480);

const NUM_PIXELS: usize = LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as usize
    * LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as usize;

use slint::platform::software_renderer::{self, TargetPixel as _};

pub type TargetPixel = ARGB8888;

#[link_section = ".frame_buffer"]
static mut FB1: [TargetPixel; NUM_PIXELS] = [TargetPixel {
    a: 0,
    r: 0,
    g: 0,
    b: 0,
}; NUM_PIXELS];

#[link_section = ".frame_buffer"]
static mut FB2: [TargetPixel; NUM_PIXELS] = [TargetPixel {
    a: 0,
    r: 0,
    g: 0,
    b: 0,
}; NUM_PIXELS];

use embedded_alloc::Heap;

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

struct StmBackendInner<'a> {
    scb: cortex_m::peripheral::SCB,
    delay: embassy_time::Delay,
    reset: Output<'a>,
    touch_screen: Ft6x36<I2c<'a, I2C1, Blocking>>,
    //fb1: &'a mut [TargetPixel; NUM_PIXELS],
    //fb2: &'a mut [TargetPixel; NUM_PIXELS],
}

struct StmBackend<'a> {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
    exti_input: ExtiInput<'a>,
    inner: RefCell<StmBackendInner<'a>>,
}

impl<'a> StmBackend<'a> {
    fn init(
        dsihost: DSIHOST,
        ltdc: LTDC,
        i2c1: I2C1,
        pb8: PB8,
        pb9: PB9,
        pg6: PG6,
        pj2: PJ2,
        pj5: PJ5,
        ph7: PH7,
        exti5: EXTI5,
        //fb1: &'a mut [TargetPixel; NUM_PIXELS],
        //fb2: &'a mut [TargetPixel; NUM_PIXELS],
        scb: SCB,
    ) -> Self {
        /*
           BSP_LCD_Reset() !!! ALSO RESETS TOUCHSCREEN - Necessary before using TS !!!
        */

        // According to UM for the discovery kit, PH7 is an active-low reset for the LCD and touchsensor
        let mut reset = Output::new(ph7, Level::Low, Speed::High);

        // CubeMX example waits 20 ms before de-asserting reset. Must be at least 5ms for the touch screen.
        embassy_time::block_for(embassy_time::Duration::from_millis(20));

        // Disable the reset signal and wait 140ms as in the Linux driver (CubeMX waits only 20)
        reset.set_high();
        embassy_time::block_for(embassy_time::Duration::from_millis(140));

        // BEGIN TOUCHSCREEN

        // "Time of starting to report point after resetting" according to TS datasheet is 300 ms after reset
        //embassy_time::block_for(embassy_time::Duration::from_millis(160));

        let mut i2c = embassy_stm32::i2c::I2c::new_blocking(
            i2c1,
            pb8,
            pb9,
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

        info!("Get touch event: {:#?}", touch_screen.get_touch_event());

        // END TOUCHSCREEN

        let mut led = Output::new(pg6, Level::High, Speed::Low);

        /*
        BSP_LCD_MspInit() // This will set IP blocks LTDC, DSI and DMA2D => We should be fine with what Embassy does.
         */

        let mut ltdc = Ltdc::new(ltdc);
        let dsi_config = dsi::Config::stm32f469_disco(LCD_ORIENTATION);
        let mut dsi = Dsi::new(dsihost, pj2, &dsi_config);

        // Here comes HAL_DSI_ConfigPhyTimer() in the BSP example. We have already configured it at the beginning.

        const PCPolarity: bool = false; // LTDC_PCPOLARITY_IPC == 0

        let ltdc_de_polarity: Depol = if dsi_config.de_polarity == false {
            Depol::ACTIVELOW
        } else {
            Depol::ACTIVEHIGH
        };
        let ltdc_vs_polarity: Vspol = if dsi_config.vs_polarity == false {
            Vspol::ACTIVEHIGH
        } else {
            Vspol::ACTIVELOW
        };

        let ltdc_hs_polarity: Hspol = if dsi_config.hs_polarity == false {
            Hspol::ACTIVEHIGH
        } else {
            Hspol::ACTIVELOW
        };

        /* Timing Configuration */
        let horizontal_sync: u16 = dsi_config.hsa - 1;
        let vertical_sync: u16 = dsi_config.vsa - 1;
        let accumulated_hbp: u16 = dsi_config.hsa + dsi_config.hbp - 1;
        let accumulated_vbp: u16 = dsi_config.vsa + dsi_config.vbp - 1;
        let accumulated_active_w: u16 = dsi_config.width + dsi_config.hsa + dsi_config.hbp - 1;
        let accumulated_active_h: u16 = dsi_config.height + dsi_config.vsa + dsi_config.vbp - 1;
        let total_width: u16 =
            dsi_config.width + dsi_config.hsa + dsi_config.hbp + dsi_config.hfp - 1;
        let total_height: u16 =
            dsi_config.height + dsi_config.vsa + dsi_config.vbp + dsi_config.vfp - 1;

        /*
        ######################################
         BEGIN HAL_LTDC_Init()
        ######################################
        */

        // DISABLE LTDC before making changes
        ltdc.disable();

        // Configure the HS, VS, DE and PC polarity
        LTDC.gcr().modify(|w| {
            w.set_hspol(ltdc_hs_polarity);
            w.set_vspol(ltdc_vs_polarity);
            w.set_depol(ltdc_de_polarity);
            w.set_pcpol(Pcpol::RISINGEDGE); // #define LTDC_PCPOLARITY_IPC 0x00000000U
        });

        // Set Synchronization size
        LTDC.sscr().modify(|w| {
            w.set_hsw(horizontal_sync);
            w.set_vsh(vertical_sync)
        });

        // Set Accumulated Back porch
        LTDC.bpcr().modify(|w| {
            w.set_ahbp(accumulated_hbp);
            w.set_avbp(accumulated_vbp);
        });

        // Set Accumulated Active Width
        LTDC.awcr().modify(|w| {
            w.set_aah(accumulated_active_h);
            w.set_aaw(accumulated_active_w);
        });

        // Set Total Width
        LTDC.twcr().modify(|w| {
            w.set_totalh(total_height);
            w.set_totalw(total_width);
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

        ltdc::config_layer(
            ltdc::Window {
                x0: 0,
                x1: LCD_DIMENSIONS.get_width(LCD_ORIENTATION),
                y0: 0,
                y1: LCD_DIMENSIONS.get_height(LCD_ORIENTATION),
            },
            ltdc::Image {
                width: LCD_DIMENSIONS.get_width(LCD_ORIENTATION),
                height: LCD_DIMENSIONS.get_height(LCD_ORIENTATION),
            },
            Pf::ARGB8888,
            255,
            0,
            ltdc::RGB {
                red: 0,
                green: 0,
                blue: 0,
            },
            unsafe { &FRAMEBUFFER[0] as *const _ as u32 },
        );

        /*
        ######################################
         END HAL_LTDC_ConfigLayer()  and LTDC_SetConfig() for Layer 0
        ######################################
        */

        let mut delay = embassy_time::Delay;

        Self {
            window: RefCell::default(),
            exti_input: embassy_stm32::exti::ExtiInput::new(pj5, exti5, Pull::None),
            inner: RefCell::new(StmBackendInner {
                scb: scb,
                delay,
                reset,
                touch_screen,
                //fb1,
                //fb2,
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

        //let (fb1, fb2) = (&mut inner.fb1, &mut inner.fb2);

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
                    info!("start rendering");
                    renderer.render(work_fb, LCD_DIMENSIONS.get_width(LCD_ORIENTATION).into());
                    inner.scb.clean_dcache_by_slice(work_fb); // Unsure... DCache and Ethernet may cause issues.

                    ltdc::set_framebuffer(work_fb.as_ptr() as *const u32);
                    core::mem::swap::<&mut [_]>(&mut work_fb, &mut displayed_fb);
                });

                // handle touch event
                let button = slint::platform::PointerEventButton::Left;
                let event = match inner
                    .touch_screen
                    .get_touch_event()
                    .map(|x| x.p1)
                    .ok()
                    .flatten()
                {
                    Some(state) => {
                        let position = slint::PhysicalPosition::new(state.y as i32, state.x as i32)
                            .to_logical(window.scale_factor());

                        info!("Got Touch: {:#?}", state);
                        Some(match last_touch.replace(position) {
                            Some(_) => slint::platform::WindowEvent::PointerMoved { position },
                            None => {
                                info!("Send Pointer Pressed");
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
        p.PF5,  // A5
        p.PF12, // A6
        p.PF13,
        p.PF14,
        p.PF15, // A9
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
        p.PH2,  // SDCKE0
        p.PG8,  // SDCLK
        p.PG15, // SDNCAS
        p.PH3,  // SDNE0
        p.PF11, // SDNRAS
        p.PC0,  // SDNWE
        embedded_dal::drivers::is42s32400f::Is42s32400f6 {},
    );

    // It has 128 Mbit / 16 MByte of RAM. 4 banks, 1M (=1024 K) x 32 bits
    // STM32F469 DISCO has 4096 rows by 256 columns by 32 bits on 4 banks
    //let sdram_size = 4096 * 256 * 4 * 4;
    let sdram_size = 4096 * 256 * 4;

    let mut delay = embassy_time::Delay;

    let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

    let ram_slice = unsafe {
        info!("RAM ADDR: {:x}", ram_ptr as u32);

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u32>())
    };

    // // ----------------------------------------------------------
    // // Use memory in SDRAM
    info!("RAM contents before writing: {:x}", ram_slice[..10]);

    ram_slice[..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    ram_slice[ram_slice.len() - 1] = 5;

    let test_slice = &[1u32, 2, 3, 4, 5, 6, 7, 8, 9, 10];

    info!("RAM contents after writing: {:x}", unsafe {
        core::ptr::read_volatile(ram_slice[..200].as_ptr() as *const [u32; 200])
    });

    info!("Test Slice: {:x}", unsafe {
        core::ptr::read_volatile(test_slice[..10].as_ptr() as *const [u32; 10])
    });

    crate::assert_eq!(ram_slice[0], 1);
    crate::assert_eq!(ram_slice[ram_slice.len() - 1], 5);

    embassy_time::block_for(Duration::from_millis(100));
    cortex_m::asm::dsb();

    let (fb1, fb2) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(FB1),
            &mut *core::ptr::addr_of_mut!(FB2),
        )
    };

    warn!("fb1 addr: {}", fb1.as_ptr());
    warn!("fb2 addr: {}", fb2.as_ptr());

    info!("before copy: fb1[0..20]: {:#?}", fb1[0..20]);

    for (index, item) in FRAMEBUFFER[..800].iter().enumerate() {
        fb1[index] = *item;
    }

    //fb1[0..20].copy_from_slice(&FRAMEBUFFER[0..20]);
    //ram_slice[0..800].copy_from_slice( (unsafe {&* (FRAMEBUFFER[..800].as_ptr() as *const [u32; 800])}));

    cortex_m::asm::dsb();

    info!("FRAMEBUFFER[0..20]: {:#?}", &FRAMEBUFFER[0..20]);
    info!("fb1[0..20]: {:#?}", fb1[0..20]);
    info!("fb1[260..280]: {:#?}", fb1[260..280]);

    embassy_time::block_for(Duration::from_millis(100));

    info!("Erasing RAM...");
    ram_slice.fill(0x00);
    info!("Done!");

    //let (heap, rest) = ram_slice.split_at_mut(HEAP_SIZE);
    //let rest = ram_slice;
    //let (fb1, rest) = rest.split_at_mut(FB_SIZE_BYTES);
    //let (fb2, rest) = rest.split_at_mut(FB_SIZE_BYTES);

    //fb2[..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

    //info!("fb2[..10]: {:x}", fb2[..10]);

    // Safe: This is an STM32L072, which has a Cortex-M0+ with MPU.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SCB.disable_dcache(&mut cp.CPUID);

    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    /*
    unsafe { ALLOCATOR.init(heap.as_mut_ptr() as usize, HEAP_SIZE) }
    const FB_SIZE_BYTES: usize = NUM_PIXELS * core::mem::size_of::<ARGB8888>();
    let (fb1, fb2) = unsafe {
        (
            core::slice::from_raw_parts_mut(fb1.as_mut_ptr() as *mut ARGB8888, NUM_PIXELS)
                .try_into()
                .unwrap(),
            core::slice::from_raw_parts_mut(fb2.as_mut_ptr() as *mut ARGB8888, NUM_PIXELS)
                .try_into()
                .unwrap(),
        )
    };
    */

    slint::platform::set_platform(Box::new(StmBackend::init(
        p.DSIHOST, p.LTDC, p.I2C1, p.PB8, p.PB9, p.PG6, p.PJ2, p.PJ5, p.PH7,
        p.EXTI5, //fb1, fb2,
        cp.SCB,
    )))
    .expect("backend already initialized");

    let window = MainWindow::new().unwrap();
    window.run().unwrap();
}

slint::include_modules!();

#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
struct ARGB8888 {
    b: u8,
    g: u8,
    r: u8,
    a: u8,
}

impl ARGB8888 {
    const fn new(a: u8, r: u8, g: u8, b: u8) -> Self {
        Self { b, g, r, a }
    }

    const fn new_rgb(r: u8, g: u8, b: u8) -> Self {
        Self::new(0xFF, r, g, b)
    }
}

use slint::platform::software_renderer::PremultipliedRgbaColor;

impl slint::platform::software_renderer::TargetPixel for ARGB8888 {
    fn blend(&mut self, color: PremultipliedRgbaColor) {
        let a = (u8::MAX - color.alpha) as u16;
        self.r = (self.r as u16 * a / 255) as u8 + color.red;
        self.g = (self.g as u16 * a / 255) as u8 + color.green;
        self.b = (self.b as u16 * a / 255) as u8 + color.blue;
        self.a = 0xFF;
    }

    fn from_rgb(r: u8, g: u8, b: u8) -> Self {
        Self::new(0xFF, r, g, b)
    }
}

//#[repr(packed(1))]
#[repr(align(1))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default)]
struct RGB888 {
    b: u8,
    g: u8,
    r: u8,
}

impl RGB888 {
    const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { b, g, r }
    }
}

impl slint::platform::software_renderer::TargetPixel for RGB888 {
    fn blend(&mut self, color: PremultipliedRgbaColor) {
        let a = (u8::MAX - color.alpha) as u16;
        self.r = (self.r as u16 * a / 255) as u8 + color.red;
        self.g = (self.g as u16 * a / 255) as u8 + color.green;
        self.b = (self.b as u16 * a / 255) as u8 + color.blue;
    }

    fn from_rgb(r: u8, g: u8, b: u8) -> Self {
        Self::new(r, g, b)
    }
}

static FRAMEBUFFER: [TargetPixel; 2 * 800] = [
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 255),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
    TargetPixel::new_rgb(0, 0, 0),
];
