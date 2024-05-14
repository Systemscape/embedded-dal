use cortex_m::asm;
use embassy_stm32::pac::ltdc::vals::{Bf1, Bf2, Imr, Pf, Vbr};
use embassy_stm32::pac::LTDC;

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

pub fn config_layer(
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

    config_window(window, pixel_format);

    config_image(image, pixel_format);

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

    apply_config();
}

pub fn config_image(image: Image, pixel_format: Pf) {
    // Configures the color frame buffer pitch in byte
    // IMPORTANT: This when splitt into two calls to modify() make sure to reload first (SRCR->IMR)!
    LTDC.layer(0).cfblr().modify(|w| {
        w.set_cfbp(image.width * pixel_format_to_size(pixel_format) as u16);
    });

    // Configures the frame buffer line number
    LTDC.layer(0)
        .cfblnr()
        .write(|w| w.set_cfblnbr(image.height));

    apply_config();
}

pub fn config_window(window: Window, pixel_format: Pf) {
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
    // IMPORTANT: This when splitt into two calls to modify() make sure to reload first (SRCR->IMR)!
    LTDC.layer(0).cfblr().modify(|w| {
        w.set_cfbll(((window.x1 - window.x0) * pixel_format_to_size(pixel_format) as u16) + 3);
    });

    apply_config();
}

fn apply_config() {
    LTDC.srcr().modify(|w| w.set_vbr(Vbr::RELOAD));

    // Ensure the modification happens before reading.
    cortex_m::asm::dsb();

    // Wait for config to be applied
    while LTDC.srcr().read().vbr() == Vbr::RELOAD {
        asm::nop();
    }
}

pub fn set_framebuffer(address: *const u32,) {
    // Configure the frame buffer start address
    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(address as u32));

    defmt::info!("Setting cfbadd: {:x}", address);

    apply_config();
}
