#![no_std]
#![no_main]

extern crate alloc;
use alloc::{borrow::ToOwned, boxed::Box, rc::Rc};
use embassy_futures::select::{select, Select};
use core::{
    cell::RefCell,
    sync::atomic::{AtomicI32, AtomicU8},
};
use cortex_m::peripheral::SCB;
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    i2c,
    i2c::I2c,
    mode::Async,
    peripherals,
    peripherals::{DSIHOST, EXTI5, LTDC, PG6, PH7, PJ2, PJ5},
    qspi::{
        enums::{
            AddressSize, ChipSelectHighTime, DummyCycles, FIFOThresholdLevel, MemorySize, QspiWidth,
        },
        Config, TransferConfig,
    },
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllRDiv, PllSource, Sysclk,
    },
    time::{mhz, Hertz},
};
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        NoopMutex,
    },
    signal::Signal,
};
use embassy_time::Duration;
use embedded_dal::{
    config::{Dimensions, Orientation},
    drivers::{
        dsi::{self, Dsi},
        ft6x36::Ft6x36,
        ltdc::{self, Ltdc},
    },
};
use embedded_hal_async::delay::DelayNs;
use pas_co2_rs::{
    regs::{MeasurementMode, OperatingMode},
    PasCo2,
};
use slint::{
    platform::{software_renderer::MinimalSoftwareWindow},
    ComponentHandle, Weak,
};
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

pub type TargetPixel = embedded_dal::pixels::ARGB8888;

const LCD_ORIENTATION: Orientation = embedded_dal::config::Orientation::Landscape;
const LCD_DIMENSIONS: Dimensions = Dimensions::from(800, 480);

const NUM_PIXELS: usize = LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as usize
    * LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as usize;

use embedded_alloc::Heap;

const HEAP_SIZE: usize = 250 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static TIME_LAST: AtomicI32 = AtomicI32::new(-1);
static SCREEN_BRIGHTNESS: AtomicU8 = AtomicU8::new(100);
static START_MEASUREMENT_SIGNAL: Signal<CriticalSectionRawMutex, Co2SensorCommand> = Signal::new();
static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async>>>> = StaticCell::new();

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

enum Co2SensorCommand {
    StartMeasurement,
    ForceCalibration(i16),
}

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

struct StmBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

struct StmBackendInner<'a> {
    scb: cortex_m::peripheral::SCB,
    /// Keep the reset pin in a defined state! (Don't `drop()` it after the init function)
    _reset: Output<'a>,
    ltdc: Ltdc<'a>,
    dsi: Dsi<'a>,
    touch_screen: Ft6x36<I2cDevice<'a, NoopRawMutex, I2c<'static, Async>>>,
    exti: ExtiInput<'a>,
}

impl<'a> StmBackendInner<'a> {
    fn init(
        dsihost: DSIHOST,
        ltdc: LTDC,
        i2c: I2cDevice<'a, NoopRawMutex, I2c<'static, Async>>,
        _pg6: PG6,
        pj2: PJ2,
        pj5: PJ5,
        ph7: PH7,
        exti5: EXTI5,
        scb: SCB,
    ) -> StmBackendInner<'a> {
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

        let mut touch_screen = embedded_dal::drivers::ft6x36::Ft6x36::new(
            i2c,
            0x38,
            embedded_dal::drivers::ft6x36::Dimension {
                x: LCD_DIMENSIONS.get_height(LCD_ORIENTATION),
                y: LCD_DIMENSIONS.get_width(LCD_ORIENTATION),
            },
        );

        debug!("Init touch_screen");
        unwrap!(touch_screen.init());
        touch_screen.set_orientation(match LCD_ORIENTATION {
            Orientation::Landscape => embedded_dal::drivers::ft6x36::Orientation::Landscape,
            Orientation::Portrait => embedded_dal::drivers::ft6x36::Orientation::Portrait,
        });

        match touch_screen.get_info() {
            Some(info) => info!("Got touch screen info: {:#?}", info),
            None => warn!("No info"),
        }

        let diag = unwrap!(touch_screen.get_diagnostics());
        info!("Got touch screen diag: {:#?}", diag);

        info!("Get touch event: {:#?}", touch_screen.get_touch_event());

        // END TOUCHSCREEN

        //let mut led = Output::new(pg6, Level::High, Speed::Low); // Currently unused

        /*
        BSP_LCD_MspInit() // This will set IP blocks LTDC, DSI and DMA2D => We should be fine with what Embassy does.
         */

        let dsi_config = dsi::Config::stm32f469_disco(LCD_ORIENTATION);

        let mut ltdc = ltdc::Ltdc::new(ltdc, &dsi_config);

        let mut dsi = Dsi::new(dsihost, pj2, &dsi_config);

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
        let mut write_closure = |address: u8, data: &[u8]| unwrap!(dsi.write_cmd(0, address, data));

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

        ltdc.config_layer(
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
            embassy_stm32::pac::ltdc::vals::Pf::ARGB8888,
            255,
            0,
            ltdc::RGB {
                red: 0,
                green: 0,
                blue: 0,
            },
            unsafe { &FB1[0] as *const _ as u32 },
        );

        /*
        ######################################
         END HAL_LTDC_ConfigLayer()  and LTDC_SetConfig() for Layer 0
        ######################################
        */

        Self {
            scb,
            _reset: reset,
            ltdc,
            dsi,
            touch_screen,
            exti: embassy_stm32::exti::ExtiInput::new(pj5, exti5, Pull::None),
        }
    }
}

impl slint::platform::Platform for StmBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        //let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(slint::platform::software_renderer::RepaintBufferType::SwappedBuffers);
        //self.window.replace(Some(window.clone()));
        //Ok(window)
        Ok(self.window.borrow_mut().clone().unwrap().clone())
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
async fn main(spawner: Spawner) {
    /*
        SystemClock_Config()
    */

    let config = get_board_config();

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
    let sdram_size = 4096 * 256 * 4 * 4;

    let ram_ptr: *mut u32 = sdram.init(&mut embassy_time::Delay) as *mut _;

    let ram_slice = unsafe {
        info!("RAM ADDR: {:x}", ram_ptr as u32);

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u32>())
    };

    // // ----------------------------------------------------------
    // // Use memory in SDRAM
    info!("RAM contents before writing: {:x}", ram_slice[..10]);

    let test_slice: &[u32; 10] = &[1u32, 2, 3, 4, 5, 6, 7, 8, 9, 10];

    ram_slice[..10].copy_from_slice(test_slice);
    ram_slice[ram_slice.len() - 1] = 0x12; // Ensure we can write until the end

    info!("RAM contents after writing: {:x}", unsafe {
        core::ptr::read_volatile(ram_slice[..20].as_ptr() as *const [u32; 20])
    });

    crate::assert_eq!(ram_slice[..10], test_slice[..10]);
    crate::assert_eq!(ram_slice[ram_slice.len() - 1], 0x12); // Ensure we can read until the end

    info!("Erasing RAM...");
    ram_slice.fill(0x00);
    info!("Done!");

    let config = Config {
        memory_size: MemorySize::_128MiB,
        address_size: AddressSize::_24bit, // 24 bit according to STM32469discovery BSP
        prescaler: 1, // Max 90 MHz. QSPI is on AHB3 and that is set to 180 MHz so divided by 2 that's 90 MHz. TODO: Make this depend on the actualy frequency? How to obtain it from embassy?
        cs_high_time: ChipSelectHighTime::_5Cycle, // 5 cycles according to STM32469discovery BSP
        fifo_threshold: FIFOThresholdLevel::_1Bytes, // 1 byte according to STM32469discovery BSP
    };

    // MT25QL128ABA1EW9-0SIT according to schetmatic
    let mut qspi_flash = embassy_stm32::qspi::Qspi::new_bank1(
        p.QUADSPI, p.PF8, p.PF9, p.PF7, p.PF6, p.PF10, p.PB6, p.DMA2_CH7, config,
    );

    let mut buf = [0u8; 8];
    let transaction = TransferConfig {
        iwidth: QspiWidth::SING,
        awidth: QspiWidth::SING,
        dwidth: QspiWidth::QUAD,
        instruction: 0x6B,
        address: Some(0x00000000),
        dummy: DummyCycles::_8,
    };
    qspi_flash.blocking_read(&mut buf, transaction);

    warn!("Read QSPI bytes: {:#?}", buf);

    embassy_time::block_for(Duration::from_millis(2000));

    extern "C" {
        static mut __s_slint_assets: u8;
        static __e_slint_assets: u8;
        static __si_slint_assets: u8;
    }

    unsafe {
        warn!(
            "__s_slint_assets: {:x}",
            core::ptr::addr_of!(__s_slint_assets) as usize
        );
        warn!(
            "__e_slint_assets: {:x}",
            core::ptr::addr_of!(__e_slint_assets) as usize
        );
        warn!(
            "__si_slint_assets: {:#x}",
            core::ptr::addr_of!(__si_slint_assets) as usize
        );

        let asset_mem_slice = core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(__s_slint_assets),
            core::ptr::addr_of!(__e_slint_assets) as usize
                - core::ptr::addr_of!(__s_slint_assets) as usize,
        );

        let mut asset_flash_addr = core::ptr::addr_of!(__si_slint_assets) as usize - 0x9000_0000;

        defmt::assert!(asset_mem_slice.len() > 0);

        for chunk in asset_mem_slice.chunks_mut(32) {
            let transaction = TransferConfig {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::SING,
                dwidth: QspiWidth::QUAD,
                instruction: 0x6B,
                address: Some(asset_flash_addr as u32),
                dummy: DummyCycles::_8,
            };

            qspi_flash.blocking_read(chunk, transaction);

            asset_flash_addr += chunk.len();
        }
    }

    let mut cp = unwrap!(cortex_m::Peripherals::take());
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

    let mut config = embassy_stm32::i2c::Config::default();
    config.sda_pullup = true;
    config.scl_pullup = true;

    let i2c = embassy_stm32::i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(400_000),
        config,
    );

    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    let i2c_dev1 = I2cDevice::new(i2c_bus);

    let inner = StmBackendInner::init(
        p.DSIHOST, p.LTDC, i2c_dev1, p.PG6, p.PJ2, p.PJ5, p.PH7, p.EXTI5, //fb1, fb2,
        cp.SCB,
    );

    // ### Init Backend ###
    info!("Init StmBackend");
    let sw_window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(StmBackend {
        window: RefCell::new(Some(sw_window.clone())),
    }))
    .expect("backend already initialized");

    let window = MainWindow::new().unwrap();

    let window_weak = window.as_weak();
    spawner
        .spawn(measure_co2(I2cDevice::new(i2c_bus), window_weak.clone()))
        .unwrap();
    //slint::spawn_local(measure_co2(I2cDevice::new(i2c_bus), window_weak.clone()));


    window.on_start_measurement(|| {
        START_MEASUREMENT_SIGNAL.signal(Co2SensorCommand::StartMeasurement)
    });

    window.on_force_compensation(move || {
        error!("on_force_compensation");
        START_MEASUREMENT_SIGNAL.signal(Co2SensorCommand::ForceCalibration(500))
    });

    window.on_brightness_increase(move || {
        error!("on_brightness_increase");
        let current = SCREEN_BRIGHTNESS.load(core::sync::atomic::Ordering::Acquire);
        if current < 240 {
            SCREEN_BRIGHTNESS.store(current + 10, core::sync::atomic::Ordering::Release);
        }
    });

    window.on_brightness_decrease(move || {
        error!("on_brightness_decrease");
        let current = SCREEN_BRIGHTNESS.load(core::sync::atomic::Ordering::Acquire);
        if current > 20 {
            SCREEN_BRIGHTNESS.store(current - 10, core::sync::atomic::Ordering::Release);
        }
    });

    let window_weak = window.as_weak();

    let co2_measurement_timer = slint::Timer::default();
    co2_measurement_timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_secs(60),
        move || window_weak.clone().unwrap().invoke_start_measurement(),
    );


    let window_weak = window.as_weak();
    let time_since_last = slint::Timer::default();
    time_since_last.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_secs(1),
        move || {
            error!("second update timer");
            let window = unwrap!(window_weak.clone().upgrade());
            let time_last = TIME_LAST.load(core::sync::atomic::Ordering::Relaxed);
            let time_since_last = embassy_time::Instant::now().as_secs() as i32 - time_last;
            window.set_time_since_last(time_since_last);
        },
    );

    window.show().unwrap();
    slint_event_loop(sw_window, inner).await;

    window.hide().unwrap();

    //window.run().unwrap();
}

#[embassy_executor::task]
async fn measure_co2(
    i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>,
    window_weak: Weak<MainWindow>,
) {
    //let pas_co2 = pas_co2_stat.init(PasCo2::new(embedded_hal_bus::i2c::RefCellDevice::new(i2c)));
    let mut pas_co2 = PasCo2::new(i2c);

    info!("Status: {}", unwrap!(pas_co2.get_status()));

    let mut mode = MeasurementMode::default();
    mode.operating_mode = OperatingMode::Idle;
    unwrap!(pas_co2.set_measurement_mode(mode));

    let pressure: u16 = 950; //hPa

    unwrap!(pas_co2.set_pressure_compensation(pressure));

    embassy_time::Delay {}.delay_ms(1000).await;

    loop {
        info!("Entering measure_co2 loop");
        match START_MEASUREMENT_SIGNAL.wait().await {
            Co2SensorCommand::ForceCalibration(val) => {
                warn!("Yielding!");
                embassy_futures::yield_now().await;
                warn!("Performing forced compensation!");
                //pas_co2                    .do_forced_compensation(val, embassy_time::Delay)                    .unwrap();
            }
            Co2SensorCommand::StartMeasurement => {
                info!("Starting Measurement");
                // Check if the last measurement has been more than 10 seconds ago to prevent measuring more often than specified by user touch.
                let time_last = TIME_LAST.load(core::sync::atomic::Ordering::Relaxed);
                let time_since_last = embassy_time::Instant::now().as_secs() as i32 - time_last;
                if time_since_last < 10 && time_last != -1 {
                    warn!("Last measurement less than 10 sec ago. Not measuring this time!");
                    continue;
                }

                let window = unwrap!(window_weak.upgrade());

                let status = unwrap!(pas_co2.get_status());
                info!("Status: {}", status);

                unwrap!(pas_co2.clear_status());

                let status_text = "Ready: ".to_owned()
                    + if status.ready { "Yes" } else { "No" }
                    + ", Temp.: "
                    + if status.temperature_error {
                        "Error"
                    } else {
                        "OK"
                    }
                    + ", Voltage: "
                    + if status.voltage_error { "Error" } else { "OK" }
                    + ", Comm.: "
                    + if status.communication_error {
                        "Error"
                    } else {
                        "OK"
                    };

                window.set_status_text(status_text.into());

                unwrap!(pas_co2.start_measurement());
                TIME_LAST.store(
                    embassy_time::Instant::now().as_secs() as i32,
                    core::sync::atomic::Ordering::Relaxed,
                );
                embassy_time::Delay.delay_ms(1150).await;

                let co2_ppm = unwrap!(pas_co2.get_co2_ppm());

                info!("CO2 PPM: {}", co2_ppm);
                window.set_co2_ppm(co2_ppm.into());

                let color = match co2_ppm {
                    0..=700 => slint::Color::from_rgb_u8(0, 200, 0),
                    701..=1200 => slint::Color::from_rgb_u8(255, 165, 0),
                    _ => slint::Color::from_rgb_u8(255, 0, 0),
                };
                window.set_background_color(color);
            }
        }
    }
}

//#[embassy_executor::task]
//async fn slint_event_loop(
async fn slint_event_loop(sw_window: Rc<MinimalSoftwareWindow>, mut inner: StmBackendInner<'static>) -> ! {
    info!("Entering slint_event_loop");
    //let inner = &mut *self.inner.borrow_mut();

    // TODO!!!

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
    sw_window.set_size(slint::PhysicalSize::new(
        LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as u32,
        LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as u32,
    ));

    sw_window.request_redraw();

    loop {
        slint::platform::update_timers_and_animations();

        sw_window.draw_if_needed(|renderer| {
            info!("start rendering...");
            renderer.render(work_fb, LCD_DIMENSIONS.get_width(LCD_ORIENTATION).into());
            info!("... done rendering");
            inner.scb.clean_dcache_by_slice(work_fb); // Unsure... DCache and Ethernet may cause issues.

            inner.ltdc.set_framebuffer(work_fb.as_ptr() as *const u32);

            core::mem::swap::<&mut [_]>(&mut work_fb, &mut displayed_fb);

            let screen_brightness = SCREEN_BRIGHTNESS.load(core::sync::atomic::Ordering::Acquire);
            info!("Setting screen brightness to {}", screen_brightness);

            let write_closure =
                |address: u8, data: &[u8]| unwrap!(inner.dsi.write_cmd(0, address, data));

            embedded_dal::drivers::nt35510::set_brightness(write_closure, screen_brightness);
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
                let position = slint::PhysicalPosition::new(state.x as i32, state.y as i32)
                    .to_logical(sw_window.scale_factor());

                //info!("Got Touch: {:#?}", state);
                Some(match last_touch.replace(position) {
                    Some(_) => slint::platform::WindowEvent::PointerMoved { position },
                    None => slint::platform::WindowEvent::PointerPressed { position, button },
                })
            }
            None => last_touch
                .take()
                .map(|position| slint::platform::WindowEvent::PointerReleased { position, button }),
        };

        if let Some(event) = event {
            let is_pointer_release_event =
                matches!(event, slint::platform::WindowEvent::PointerReleased { .. });

                sw_window.dispatch_event(event);

            // removes hover state on widgets^
            if is_pointer_release_event {
                sw_window.dispatch_event(slint::platform::WindowEvent::PointerExited);
            }
            continue;
        }

        // Try to put the MCU to sleep
        if !sw_window.has_active_animations() {
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                info!("waiting for: {}", duration);
                select(embassy_time::Timer::after(duration.try_into().unwrap()), inner.exti.wait_for_any_edge()).await;
                continue;
            }
            inner.exti.wait_for_any_edge().await;
            
        }
    }
}

slint::include_modules!();

#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::asm::delay(500_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

fn get_board_config() -> embassy_stm32::Config {
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
    config
}
