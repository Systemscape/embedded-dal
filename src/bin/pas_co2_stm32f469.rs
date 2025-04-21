#![no_std]
#![no_main]

extern crate alloc;
use alloc::{borrow::ToOwned, boxed::Box, rc::Rc};
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_futures::select::select;
use embassy_stm32::interrupt;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    i2c::{self, I2c},
    interrupt::{InterruptExt, Priority},
    ltdc::LtdcLayerConfig,
    mode::Async,
    peripherals::{self, QUADSPI},
    qspi::{
        enums::{
            AddressSize, ChipSelectHighTime, DummyCycles, FIFOThresholdLevel, MemorySize, QspiWidth,
        },
        Config, Qspi, TransferConfig,
    },
    rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllRDiv, PllSource, Sysclk,
    },
    time::{mhz, Hertz},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_dal::{
    backends::{double_buffer_embassy::DoubleBuffer, slint_embassy::Backend},
    config::{Dimensions, Orientation},
    drivers::{
        dsi::{self, Dsi},
        ft6x36::{self, Ft6x36},
        ltdc::{self, Ltdc},
    },
    pixels::ARGB8888,
};
use embedded_hal_async::delay::DelayNs;
use pas_co2_rs::{
    regs::{MeasurementMode, OperatingMode},
    PasCo2,
};
use slint::platform::software_renderer::RenderingRotation;
use slint::{
    platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
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

use embedded_alloc::LlffHeap as Heap;

const HEAP_SIZE: usize = 250 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static TIME_LAST: AtomicI32 = AtomicI32::new(-1);
static SCREEN_CONTROL_SIGNAL: Channel<CriticalSectionRawMutex, ScreenControlCommand, 20> =
    Channel::new();

static ROTATE_SCREEN: AtomicBool = AtomicBool::new(false);

static TRIGGER_START_MEASUREMENT: Channel<CriticalSectionRawMutex, Co2SensorCommand, 2> =
    Channel::new();
static TRIGGER_RENDERER: Channel<
    CriticalSectionRawMutex,
    Option<slint::platform::WindowEvent>,
    20,
> = Channel::new();
static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> = StaticCell::new();

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

enum ScreenControlCommand {
    BrightnessIncrease,
    BrightnessDecrease,
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn UART5() {
    EXECUTOR_MED.on_interrupt()
}

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    LTDC => embassy_stm32::ltdc::InterruptHandler<peripherals::LTDC>;
});

struct Stm32F469IDisco<'a> {
    pub ltdc: Ltdc<'a>,
    pub dsi: Dsi<'a>,
    pub qspi_flash: Qspi<'a, QUADSPI, Async>,
    pub i2c_bus: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async>>,
    //pub touch_screen: Ft6x36<I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, Async>>>,
    pub exti: ExtiInput<'a>,
    pub fb1: &'a mut [ARGB8888; NUM_PIXELS],
    pub fb2: &'a mut [ARGB8888; NUM_PIXELS],
    /// Keep the reset pin in a defined state! (Don't `drop()` it after the init function)
    _reset: Output<'a>,
}

impl<'a> Stm32F469IDisco<'a> {
    #[allow(clippy::too_many_arguments)]
    fn init(/*
        dsihost: DSIHOST,
        ltdc: LTDC,
        i2c: I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, Async>>,
        _pg6: PG6,
        pj2: PJ2,
        pj5: PJ5,
        ph7: PH7,
        exti5: EXTI5,
        fb1: &'a mut [ARGB8888; NUM_PIXELS],
        fb2: &'a mut [ARGB8888; NUM_PIXELS],
        scb: SCB,
        */) -> Stm32F469IDisco<'a> {
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

        info!("Erasing RAM...");
        ram_slice.fill(0x00);
        info!("Done!");

        // Safety: This must only be called once after initializing the RAM
        let (fb1, fb2) = unsafe {
            (
                &mut *core::ptr::addr_of_mut!(FB1),
                &mut *core::ptr::addr_of_mut!(FB2),
            )
        };

        let qspi_flash_config = Config {
            memory_size: MemorySize::_128MiB,
            address_size: AddressSize::_24bit, // 24 bit according to STM32469discovery BSP
            prescaler: 1, // Max 90 MHz. QSPI is on AHB3 and that is set to 180 MHz so divided by 2 that's 90 MHz. TODO: Make this depend on the actualy frequency? How to obtain it from embassy?
            cs_high_time: ChipSelectHighTime::_5Cycle, // 5 cycles according to STM32469discovery BSP
            fifo_threshold: FIFOThresholdLevel::_1Bytes, // 1 byte according to STM32469discovery BSP
        };

        // MT25QL128ABA1EW9-0SIT according to schetmatic
        let mut qspi_flash = embassy_stm32::qspi::Qspi::new_bank1(
            p.QUADSPI,
            p.PF8,
            p.PF9,
            p.PF7,
            p.PF6,
            p.PF10,
            p.PB6,
            p.DMA2_CH7,
            qspi_flash_config,
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

        let mut cp = unwrap!(cortex_m::Peripherals::take());
        cp.SCB.disable_dcache(&mut cp.CPUID);

        unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

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

        let i2c_bus = Mutex::new(i2c);
        let i2c_bus = I2C_BUS.init(i2c_bus);

        // According to UM for the discovery kit, PH7 is an active-low reset for the LCD and touchsensor
        let mut reset = Output::new(p.PH7, Level::Low, Speed::High);

        // CubeMX example waits 20 ms before de-asserting reset. Must be at least 5ms for the touch screen.
        embassy_time::block_for(embassy_time::Duration::from_millis(20));

        // Disable the reset signal and wait 140ms as in the Linux driver (CubeMX waits only 20)
        reset.set_high();
        embassy_time::block_for(embassy_time::Duration::from_millis(140));

        //let mut led = Output::new(pg6, Level::High, Speed::Low); // Currently unused

        let dsi_config = dsi::Config::stm32f469_disco(LCD_ORIENTATION);

        let mut ltdc = ltdc::Ltdc::new(p.LTDC, &dsi_config);

        let mut dsi: Dsi<'_> = Dsi::new(p.DSIHOST, p.PJ2, &dsi_config);

        dsi.enable();

        // FIXME: This should probably be done with a trait or so...
        let mut write_closure = |address: u8, data: &[u8]| unwrap!(dsi.write_cmd(0, address, data));

        embedded_dal::drivers::nt35510::init(
            &mut write_closure,
            embassy_time::Delay,
            LCD_ORIENTATION,
        );

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

        Self {
            ltdc,
            dsi,
            qspi_flash,
            i2c_bus,
            exti: embassy_stm32::exti::ExtiInput::new(p.PJ5, p.EXTI5, Pull::None),
            fb1,
            fb2,
            _reset: reset,
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    /*
        SystemClock_Config()
    */

    let mut board = Stm32F469IDisco::init();

    // BEGIN TOUCHSCREEN

    // "Time of starting to report point after resetting" according to TS datasheet is 300 ms after reset
    //embassy_time::block_for(embassy_time::Duration::from_millis(160));

    let mut touch_screen = Ft6x36::new(
        I2cDevice::new(board.i2c_bus),
        0x38,
        ft6x36::Dimension {
            x: LCD_DIMENSIONS.get_height(LCD_ORIENTATION),
            y: LCD_DIMENSIONS.get_width(LCD_ORIENTATION),
        },
    );

    debug!("Init touch_screen");
    unwrap!(embassy_futures::block_on(touch_screen.init()));
    touch_screen.set_orientation(match LCD_ORIENTATION {
        Orientation::Landscape => ft6x36::Orientation::Landscape,
        Orientation::Portrait => ft6x36::Orientation::Portrait,
    });

    match touch_screen.get_info() {
        Some(info) => info!("Got touch screen info: {:#?}", info),
        None => warn!("No info"),
    }

    let diag = unwrap!(embassy_futures::block_on(touch_screen.get_diagnostics()));
    info!("Got touch screen diag: {:#?}", diag);

    info!("Init Backend...");
    let sw_window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
    sw_window.set_size(slint::PhysicalSize::new(
        LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as u32,
        LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as u32,
    ));

    slint::platform::set_platform(Box::new(Backend::new(
        sw_window.clone(),
        &mut board.qspi_flash,
    )))
    .expect("backend already initialized");

    info!("StmBackend initialized!");

    let window = MainWindow::new().unwrap();

    window.on_start_measurement(|| {
        let _ = TRIGGER_START_MEASUREMENT.try_send(Co2SensorCommand::StartMeasurement);
    });

    window.on_force_compensation(move || {
        debug!("on_force_compensation");
        let _ = TRIGGER_START_MEASUREMENT.try_send(Co2SensorCommand::ForceCalibration(450));
    });

    window.on_brightness_increase(move || {
        debug!("on_brightness_increase");
        let _ = SCREEN_CONTROL_SIGNAL.try_send(ScreenControlCommand::BrightnessIncrease);
    });

    window.on_brightness_decrease(move || {
        debug!("on_brightness_decrease");
        let _ = SCREEN_CONTROL_SIGNAL.try_send(ScreenControlCommand::BrightnessDecrease);
    });

    window.on_rotate_screen(move || {
        debug!("on_rotate_screen");
        ROTATE_SCREEN.fetch_not(core::sync::atomic::Ordering::SeqCst);
    });

    let window_weak = window.as_weak();

    let co2_measurement_timer = slint::Timer::default();
    co2_measurement_timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_secs(60),
        move || window_weak.clone().unwrap().invoke_start_measurement(),
    );

    // Trigger one measruement, so we don't wait for 60 seconds before the first measurement.
    let _ = TRIGGER_START_MEASUREMENT.try_send(Co2SensorCommand::StartMeasurement);

    let window_weak = window.as_weak();
    let time_since_last = slint::Timer::default();
    time_since_last.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_secs(1),
        move || {
            let window = unwrap!(window_weak.clone().upgrade());
            let time_last = TIME_LAST.load(core::sync::atomic::Ordering::Relaxed);
            let time_since_last = embassy_time::Instant::now().as_secs() as i32 - time_last;
            window.set_time_since_last(time_since_last);
        },
    );

    let exti = board.exti;

    info!("Spawning Touchscreen Task");
    // High-priority executor: UART4, priority level 6
    embassy_stm32::interrupt::UART4.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(embassy_stm32::interrupt::UART4);
    unwrap!(spawner.spawn(read_touch_screen(
        touch_screen,
        exti,
        sw_window.scale_factor(),
    )));

    let window_weak = window.as_weak();

    info!("Spawning CO2 Measurement Task");
    // Medium-priority executor: UART5, priority level 7
    embassy_stm32::interrupt::UART5.set_priority(Priority::P7);
    let spawner = EXECUTOR_MED.start(embassy_stm32::interrupt::UART5);
    unwrap!(spawner.spawn(measure_co2(
        I2cDevice::new(board.i2c_bus),
        window_weak.clone()
    )));
    unwrap!(spawner.spawn(manage_dsi(board.dsi)));

    window.show().unwrap();

    info!("Spawning Remaining Tasks");
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(slint_event_loop(
            sw_window, board.ltdc, board.fb1, board.fb2
        )));
    });
}

/// Task that mostly controls screen brightness, because that is a DSI feature:
/// we need to send commands to the display controller directly
#[embassy_executor::task]
async fn manage_dsi(mut dsi: Dsi<'static>) {
    let mut screen_brightness: u8 = 100;

    let mut write_closure = |address: u8, data: &[u8]| unwrap!(dsi.write_cmd(0, address, data));

    // Increase or decrease screen brightness whenever the signal is received. But make sure it stays in the range [10, 250].
    loop {
        match SCREEN_CONTROL_SIGNAL.receive().await {
            ScreenControlCommand::BrightnessIncrease => {
                screen_brightness = screen_brightness.checked_add(10).unwrap_or(250)
            }
            ScreenControlCommand::BrightnessDecrease => {
                screen_brightness = screen_brightness.checked_sub(10).unwrap_or(10)
            }
        }

        // This ensure the Decrease cannot set it to 0. TODO: Make this more concise...
        screen_brightness = screen_brightness.clamp(10, 250);

        info!("Setting screen brightness to {}", screen_brightness);

        embedded_dal::drivers::nt35510::set_brightness(&mut write_closure, screen_brightness);
    }
}

/// Eventloop implementation
///
/// Handles rendering into the `fb1`/`fb2` doublebuffer and manages events sent via `TRIGGER_RENDERER`
#[embassy_executor::task]
async fn slint_event_loop(
    sw_window: Rc<MinimalSoftwareWindow>,
    mut ltdc: Ltdc<'static>,
    fb1: &'static mut [ARGB8888],
    fb2: &'static mut [ARGB8888],
) -> ! {
    info!("Entering slint_event_loop");

    let mut double_buffer = DoubleBuffer::new(
        fb1,
        fb2,
        LtdcLayerConfig {
            pixel_format: embassy_stm32::ltdc::PixelFormat::ARGB8888, // 2 bytes per pixel
            layer: embassy_stm32::ltdc::LtdcLayer::Layer1,
            window_x0: 0,
            window_x1: LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as _,
            window_y0: 0,
            window_y1: LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as _,
        },
    );

    // On first render or when rotating the screen, we have to render into the `NewBuffer` two times to avoid artifacts
    // caused by some unknown error with the `SwappedBuffers`
    let mut render_counter: u8 = 0;

    loop {
        // Get all events in the channel and dispatch them to the window
        while let Ok(Some(event)) = TRIGGER_RENDERER.try_receive() {
            sw_window.dispatch_event(event);
        }

        // Advance timers etc.
        slint::platform::update_timers_and_animations();

        // Blocking render
        let is_dirty = sw_window.draw_if_needed(|renderer| {
            let rotate_screen = ROTATE_SCREEN.load(Ordering::SeqCst);

            // Rotate the screen if the current rotation does not match the selected one
            if rotate_screen && renderer.rendering_rotation() == RenderingRotation::NoRotation {
                renderer.set_rendering_rotation(RenderingRotation::Rotate180);
                sw_window.request_redraw();
                render_counter = 0;
            } else if !rotate_screen
                && renderer.rendering_rotation() == RenderingRotation::Rotate180
            {
                renderer.set_rendering_rotation(RenderingRotation::NoRotation);
                sw_window.request_redraw();
                render_counter = 0;
            }

            // Only use SwappedBuffers after 2 rendering cycles. Otherwise screen stays black or has artifacts :/
            match render_counter {
                0 => {
                    renderer.set_repaint_buffer_type(
                        slint::platform::software_renderer::RepaintBufferType::NewBuffer,
                    );
                    render_counter += 1;
                }
                1 => {
                    render_counter += 1;
                }
                2 => {
                    info!("Setting to SwappedBuffers");
                    renderer.set_repaint_buffer_type(
                        slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
                    );
                    render_counter += 1
                }
                _ => (),
            }

            debug!("Drawing...");
            let buffer = double_buffer.current();
            renderer.render(buffer, LCD_DIMENSIONS.get_width(LCD_ORIENTATION).into());
        });

        // Only swap buffers if something was drawn
        if is_dirty {
            debug!("Swapping buffers...");
            double_buffer.swap(&mut ltdc.ltdc).await.unwrap();
            debug!("DONE Swapping buffers!");
        }

        // Try to put the MCU to sleep
        if !sw_window.has_active_animations() {
            // When a duration is available, use that. Otherwise continue the loop at least every 10s.
            let duration = slint::platform::duration_until_next_timer_update()
                .unwrap_or(core::time::Duration::from_secs(10));
            debug!("waiting for: {}", duration);

            // Select whatever yields earlier (timer or trigger).
            // The event from the trigger will be dispatched at the beginning of the loop.
            let _ = select(
                Timer::after(duration.try_into().unwrap()),
                TRIGGER_RENDERER.ready_to_receive(),
            )
            .await;
        }
    }
}

/// Read the touchscreen via I2C and send events to the slint eventloop
#[embassy_executor::task]
async fn read_touch_screen(
    mut touch_screen: Ft6x36<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>,
    mut exti: ExtiInput<'static>,
    window_scale_factor: f32,
) {
    debug!("Entering read_touch_screen");
    let mut last_touch = None;

    loop {
        debug!("read_touch_screen loop begin");

        // Remember the current exti pin state. In case it changes during read or rendering.
        let exti_pin_high = exti.is_high();

        // Mousebutton we send as a click (i.e., match touch to left-click)
        let button = slint::platform::PointerEventButton::Left;

        // Handle touch event
        let event = match touch_screen
            .get_touch_event()
            .await
            .map(|x| x.p1)
            .ok()
            .flatten()
        {
            Some(state) => {
                // If the screen is rotated, we need to map the x and y positions
                let x = if !ROTATE_SCREEN.load(Ordering::SeqCst) {
                    state.x as i32
                } else {
                    LCD_DIMENSIONS.get_width(LCD_ORIENTATION) as i32 - state.x as i32
                };
                let y = if !ROTATE_SCREEN.load(Ordering::SeqCst) {
                    state.y as i32
                } else {
                    LCD_DIMENSIONS.get_height(LCD_ORIENTATION) as i32 - state.y as i32
                };

                let position = slint::PhysicalPosition::new(x, y).to_logical(window_scale_factor);

                trace!("Got Touch: {:#?}", state);
                Some(match last_touch.replace(position) {
                    Some(_) => slint::platform::WindowEvent::PointerMoved { position },
                    None => slint::platform::WindowEvent::PointerPressed { position, button },
                })
            }
            // When to touch is registered, update last_touch and send the PointerReleased event
            None => last_touch
                .take()
                .map(|position| slint::platform::WindowEvent::PointerReleased { position, button }),
        };

        // Send the event to eventloop
        if let Some(event) = event {
            // Store flag before dispatching the event
            let is_pointer_release_event =
                matches!(event, slint::platform::WindowEvent::PointerReleased { .. });

            TRIGGER_RENDERER.send(Some(event)).await;

            // Remove hover state on widgets by sending PointerExited event, as well.
            if is_pointer_release_event {
                // Trigger the rendering loop
                TRIGGER_RENDERER
                    .send(Some(slint::platform::WindowEvent::PointerExited))
                    .await;
                debug!("PointerExited sent")
            }
        }

        debug!("read_touch_screen loop end. Waiting for exti with timeout...");

        // Drop Result<>, in case of an error (i.e., timeout), we just continue the loop.
        let _ = embassy_time::with_timeout(Duration::from_secs(1), async {
            if exti_pin_high {
                exti.wait_for_low().await;
            } else {
                exti.wait_for_high().await;
            }
        })
        .await;
    }
}

/// Task that performs CO2 measurements via I2C
#[embassy_executor::task]
async fn measure_co2(
    i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    window_weak: Weak<MainWindow>,
) {
    debug!("measure_co2 task begin");

    let window = unwrap!(window_weak.upgrade());

    // Setup and configure the sensor
    let mut pas_co2 = PasCo2::new(i2c);

    info!("CO2 Sensor Status: {}", unwrap!(pas_co2.get_status().await));

    let mode: MeasurementMode = MeasurementMode {
        operating_mode: OperatingMode::Idle,
        ..Default::default()
    };
    unwrap!(pas_co2.set_measurement_mode(mode).await);

    let pressure: u16 = 950; //hPa
    unwrap!(pas_co2.set_pressure_compensation(pressure).await);

    // Wait a bit for the sensor to settle and give other tasks time to run.
    Timer::after_secs(1).await;

    loop {
        info!("Entering measure_co2 loop");
        match TRIGGER_START_MEASUREMENT.receive().await {
            Co2SensorCommand::ForceCalibration(val) => {
                window.set_in_progress("Calibrating to 450ppm... ".to_owned().into());
                window.set_ampel_value(Ampel::Calibration);

                // Make it render the changed text and color...
                TRIGGER_RENDERER.send(None).await;
                // ... and give the renderer some time to do so
                Timer::after_millis(500).await;

                warn!("Performing forced compensation!");
                pas_co2
                    .do_forced_compensation(val, embassy_time::Delay)
                    .await
                    .unwrap();

                window.set_in_progress("".to_owned().into());

                // Make it render the changed text...
                TRIGGER_RENDERER.send(None).await;
                // ... and give the renderer some time to do so
                Timer::after_millis(500).await;
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

                let status = unwrap!(pas_co2.get_status().await);
                info!("Status: {}", status);

                unwrap!(pas_co2.clear_status().await);

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

                // Make it render the changed text.
                // It will get some time to render while we wait for the measurement to complete.
                TRIGGER_RENDERER.send(None).await;

                // Trigger a measurement
                unwrap!(pas_co2.start_measurement().await);
                TIME_LAST.store(
                    embassy_time::Instant::now().as_secs() as i32,
                    core::sync::atomic::Ordering::Relaxed,
                );

                // Sensor takes around 1s to complete a measurement
                embassy_time::Delay.delay_ms(1150).await;

                // Read the measured value
                let co2_ppm = unwrap!(pas_co2.get_co2_ppm().await);

                info!("CO2 PPM: {}", co2_ppm);
                window.set_co2_ppm(co2_ppm.into());

                let color = match co2_ppm {
                    0..=700 => Ampel::Green,
                    701..=1200 => Ampel::Yellow,
                    _ => Ampel::Red,
                };
                window.set_ampel_value(color);

                // Make it render the changed text.
                TRIGGER_RENDERER.sender().send(None).await;
            }
        }
    }
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

slint::include_modules!();

/// Custom exception handler that triggers a system reset after a delay
#[cortex_m_rt::exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::asm::delay(500_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}
