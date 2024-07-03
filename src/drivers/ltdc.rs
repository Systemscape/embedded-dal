use cortex_m::asm;
use embassy_stm32::{
    pac::{
        ltdc::vals::{Bf1, Bf2, Depol, Hspol, Pcpol, Pf, Vbr, Vspol},
        LTDC,
    },
    peripherals::LTDC,
};

pub struct Window {
    pub x0: u16,
    pub x1: u16,
    pub y0: u16,
    pub y1: u16,
}

pub struct Image {
    pub width: u16,
    pub height: u16,
}

pub struct RGB {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

const fn pixel_format_to_size(pixel_format: Pf) -> u8 {
    match pixel_format {
        Pf::ARGB8888 => 4,
        Pf::RGB888 => 3,
        Pf::ARGB4444 | Pf::RGB565 | Pf::ARGB1555 | Pf::AL88 => 2,
        _ => 1,
    }
}

pub struct Ltdc<'a> {
    _ltdc: embassy_stm32::ltdc::Ltdc<'a, LTDC>,
}

impl Ltdc<'_> {
    pub fn new(ltdc: LTDC, dsi_config: &crate::drivers::dsi::Config) -> Self {
        let mut ltdc = embassy_stm32::ltdc::Ltdc::new(ltdc);

        // Here comes HAL_DSI_ConfigPhyTimer() in the BSP example. We have already configured it at the beginning.

        let ltdc_pc_polarity = Pcpol::RISINGEDGE; // LTDC_PCPOLARITY_IPC == 0

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
            w.set_pcpol(ltdc_pc_polarity); // #define LTDC_PCPOLARITY_IPC 0x00000000U
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

        Self { _ltdc: ltdc }
    }

    pub fn config_layer(
        &mut self,
        window: Window,
        image: Image,
        pixel_format: Pf,
        alpha: u8,
        alpha0: u8,
        backcolor: RGB,
        framebuffer_address: u32,
    ) {
        /*:
        ######################################
         BEGIN HAL_LTDC_ConfigLayer() and LTDC_SetConfig() for Layer 0
        ######################################
        */

        self.config_window(window, pixel_format);

        self.config_image(image, pixel_format);

        // Specify the pixel format
        LTDC.layer(0).pfcr().write(|w| w.set_pf(pixel_format));

        // Configures the default color values as zero
        LTDC.layer(0).dccr().modify(|w| {
            w.set_dcblue(backcolor.blue);
            w.set_dcgreen(backcolor.green);
            w.set_dcred(backcolor.red);
            w.set_dcalpha(alpha0);
        });

        // Specifies the constant alpha value
        LTDC.layer(0).cacr().write(|w| w.set_consta(alpha));

        // Specifies the blending factors
        LTDC.layer(0).bfcr().write(|w| {
            w.set_bf1(Bf1::CONSTANT);
            w.set_bf2(Bf2::CONSTANT);
        });

        // Configure the frame buffer start address
        LTDC.layer(0)
            .cfbar()
            .write(|w| w.set_cfbadd(framebuffer_address));

        // Enable LTDC_Layer by setting LEN bit
        LTDC.layer(0).cr().modify(|w| w.set_len(true));

        self.apply_config();
    }

    pub fn config_image(&mut self, image: Image, pixel_format: Pf) {
        // Configures the color frame buffer pitch in byte
        // IMPORTANT: This when splitt into two calls to modify() make sure to reload first (SRCR->IMR)!
        LTDC.layer(0).cfblr().modify(|w| {
            w.set_cfbp(image.width * pixel_format_to_size(pixel_format) as u16);
        });

        // Configures the frame buffer line number
        LTDC.layer(0)
            .cfblnr()
            .write(|w| w.set_cfblnbr(image.height));

        self.apply_config();
    }

    pub fn config_window(&mut self, window: Window, pixel_format: Pf) {
        // Configure the horizontal start and stop position
        LTDC.layer(0).whpcr().modify(|w| {
            w.set_whstpos(LTDC.bpcr().read().ahbp() + 1 + window.x0);
            w.set_whsppos(LTDC.bpcr().read().ahbp() + window.x1);
        });

        // Configures the vertical start and stop position
        LTDC.layer(0).wvpcr().modify(|w| {
            w.set_wvstpos(LTDC.bpcr().read().avbp() + 1 + window.y0);
            w.set_wvsppos(LTDC.bpcr().read().avbp() + window.y1);
        });

        // Configures the color frame buffer pitch in byte
        // IMPORTANT: This when splitt into two calls to modify() make sure to reload first (SRCR->IMR), i.e., apply_config()!
        LTDC.layer(0).cfblr().modify(|w| {
            w.set_cfbll(((window.x1 - window.x0) * pixel_format_to_size(pixel_format) as u16) + 3);
        });

        self.apply_config();
    }

    fn apply_config(&mut self) {
        LTDC.srcr().modify(|w| w.set_vbr(Vbr::RELOAD));

        // Ensure the modification happens before reading.
        cortex_m::asm::dsb();

        // Wait for config to be applied
        while LTDC.srcr().read().vbr() == Vbr::RELOAD {
            asm::nop();
        }
    }

    pub fn set_framebuffer(&mut self, address: *const u32) {
        // Configure the frame buffer start address
        LTDC.layer(0)
            .cfbar()
            .write(|w| w.set_cfbadd(address as u32));

        cortex_m::asm::dsb();

        defmt::trace!("Setting cfbadd: {:x}", address);

        self.apply_config();
    }
}
