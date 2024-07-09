extern crate alloc;
use core::cell::RefCell;

use alloc::rc::Rc;
use embassy_stm32::{
    mode::Async,
    peripherals::QUADSPI,
    qspi::{
        enums::{DummyCycles, QspiWidth},
        Qspi,
    },
};
use slint::platform::software_renderer::MinimalSoftwareWindow;

impl slint::platform::Platform for Backend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        /*
        // Doesn't work when using a "manual" eventloop for some reason.
        // You have to set the SwappedBuffers after 2 rendering cycles.
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
        */
        Ok(self.window.borrow_mut().clone().unwrap().clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(embassy_time::Instant::now().as_micros())
    }

    #[cfg(feature = "defmt")]
    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

pub struct Backend {
    window:
        core::cell::RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl Backend {
    pub fn new(window: Rc<MinimalSoftwareWindow>, qspi_flash: &mut Qspi<QUADSPI, Async>) -> Self {
        Self::init_assets(qspi_flash);
        Self {
            window: RefCell::new(Some(window)),
        }
    }

    fn init_assets(qspi_flash: &mut Qspi<QUADSPI, Async>) {
        extern "C" {
            static mut __s_slint_assets: u8;
            static __e_slint_assets: u8;
            static __si_slint_assets: u8;
        }

        unsafe {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "__s_slint_assets: {:x}",
                core::ptr::addr_of!(__s_slint_assets) as usize
            );
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "__e_slint_assets: {:x}",
                core::ptr::addr_of!(__e_slint_assets) as usize
            );
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "__si_slint_assets: {:#x}",
                core::ptr::addr_of!(__si_slint_assets) as usize
            );

            let asset_mem_slice = core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(__s_slint_assets),
                core::ptr::addr_of!(__e_slint_assets) as usize
                    - core::ptr::addr_of!(__s_slint_assets) as usize,
            );

            let mut asset_flash_addr =
                core::ptr::addr_of!(__si_slint_assets) as usize - 0x9000_0000;

            //defmt::assert!(asset_mem_slice.len() > 0);

            for chunk in asset_mem_slice.chunks_mut(32) {
                let transaction = embassy_stm32::qspi::TransferConfig {
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
    }
}
