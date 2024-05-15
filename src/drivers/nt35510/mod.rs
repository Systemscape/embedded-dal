#![allow(dead_code)]

use crate::config::Orientation::{self, *};
use embedded_hal::delay::DelayNs;

pub fn init(mut write: impl FnMut(u8, &[u8]), mut delay: impl DelayNs, orientation: Orientation) {
    write(0xF0, &[0x55, 0xAA, 0x52, 0x08, 0x01]); // LV2:  Page 1 enable
    write(0xB0, &[0x03, 0x03, 0x03]); // AVDD: 5.2V
    write(0xB6, &[0x46, 0x46, 0x46]); // AVDD: Ratio
    write(0xB1, &[0x03, 0x03, 0x03]); // AVEE: -5.2V
    write(0xB7, &[0x36, 0x36, 0x36]); // AVEE: Ratio
    write(0xB2, &[0x00, 0x00, 0x02]); // VCL: -2.5V
    write(0xB8, &[0x26, 0x26, 0x26]); // VCL: Ratio
    write(0xBF, &[0x01]); // VGH: 15V (Free Pump)
    write(0xB3, &[0x09, 0x09, 0x09]);
    write(0xB9, &[0x36, 0x36, 0x36]); // VGH: Ratio
    write(0xB5, &[0x08, 0x08, 0x08]); // VGL_REG: -10V
    write(0xBA, &[0x26, 0x26, 0x26]); // VGLX: Ratio

    write(0xBC, &[0x00, 0x80, 0x00]); // VGMP/VGSP: 4.5V/0V
    write(0xBD, &[0x00, 0x80, 0x00]); // VGMN/VGSN:-4.5V/0V
    write(0xBE, &[0x00, 0x50]); // VCOM: -1.325V
    write(0xF0, &[0x55, 0xAA, 0x52, 0x08, 0x00]); // LV2: Page 0 enable
    write(0xB1, &[0xFC, 0x00]); // Display control
    write(0xB6, &[0x03]); // Src hold time
    write(0xB5, &[0x51]);
    write(0xB7, &[0x00, 0x00]); // Gate EQ control
    write(0xB8, &[0x01, 0x02, 0x02, 0x02]); // Src EQ control(Mode2)
    write(0xBC, &[0x00, 0x00, 0x00]); // Inv. mode(2-dot)
    write(0xCC, &[0x03, 0x00, 0x00]);
    write(0xBA, &[0x01]);

    // Tear on
    write(NT35510_CMD_TEEON, &[0x00]); // Tear on

    // Set Pixel color format to RGB888
    write(NT35510_CMD_COLMOD, &[NT35510_COLMOD_RGB888]);

    // Add a delay, otherwise MADCTL not taken
    delay.delay_ms(200);

    // Configure orientation as landscape
    match orientation {
        Landscape => {
            write(NT35510_CMD_MADCTL, &[0x60]);
            write(NT35510_CMD_CASET, &[0x00, 0x00, 0x03, 0x1F]);
            write(NT35510_CMD_RASET, &[0x00, 0x00, 0x01, 0xDF]);
        }
        Portrait => {
            write(NT35510_CMD_MADCTL, &[0x00]);
            write(NT35510_CMD_CASET, &[0x00, 0x00, 0x01, 0xDF]);
            write(NT35510_CMD_RASET, &[0x00, 0x00, 0x03, 0x1F]);
        }
    }

    // Sleep out
    write(NT35510_CMD_SLPOUT, &[0x00]);

    // Wait for sleep out exit
    delay.delay_ms(120);

    /* CABC : Content Adaptive Backlight Control section start >> */
    /* Note : defaut is 0 (lowest Brightness), 0xFF is highest Brightness, try 0x7F : intermediate value */
    write(NT35510_CMD_WRDISBV, &[0x7F]);
    /* defaut is 0, try 0x2C - Brightness Control Block, Display Dimming & BackLight on */
    write(NT35510_CMD_WRCTRLD, &[0x2C]);
    /* defaut is 0, try 0x02 - image Content based Adaptive Brightness [Still Picture] */
    write(NT35510_CMD_WRCABC, &[0x02]);
    /* defaut is 0 (lowest Brightness), 0xFF is highest Brightness */
    write(NT35510_CMD_WRCABCMB, &[0xFF]);
    /* CABC : Content Adaptive Backlight Control section end << */
    /* Display on */
    write(NT35510_CMD_DISPON, &[0x00]);

    /* Send Command GRAM memory write (no parameters) : this initiates frame write via other DSI commands sent by */
    /* DSI host from LTDC incoming pixels in video mode */
    write(NT35510_CMD_RAMWR, &[0x00]);
}

pub fn set_brightness(mut write: impl FnMut(u8, &[u8]), value: u8) {
    write(NT35510_CMD_WRDISBV, &[value]);
}

const NT35510_WRITES_0: &[u8] = &[0x55, 0xAA, 0x52, 0x08, 0x01, 0xF0]; // LV2:  Page 1 enable
const NT35510_WRITES_1: &[u8] = &[0x03, 0x03, 0x03, 0xB0]; // AVDD: 5.2V
const NT35510_WRITES_2: &[u8] = &[0x46, 0x46, 0x46, 0xB6]; // AVDD: Ratio
const NT35510_WRITES_3: &[u8] = &[0x03, 0x03, 0x03, 0xB1]; // AVEE: -5.2V
const NT35510_WRITES_4: &[u8] = &[0x36, 0x36, 0x36, 0xB7]; // AVEE: Ratio
const NT35510_WRITES_5: &[u8] = &[0x00, 0x00, 0x02, 0xB2]; // VCL: -2.5V
const NT35510_WRITES_6: &[u8] = &[0x26, 0x26, 0x26, 0xB8]; // VCL: Ratio
const NT35510_WRITES_7: &[u8] = &[0xBF, 0x01]; // VGH: 15V (Free Pump)
const NT35510_WRITES_8: &[u8] = &[0x09, 0x09, 0x09, 0xB3];
const NT35510_WRITES_9: &[u8] = &[0x36, 0x36, 0x36, 0xB9]; // VGH: Ratio
const NT35510_WRITES_10: &[u8] = &[0x08, 0x08, 0x08, 0xB5]; // VGL_REG: -10V
const NT35510_WRITES_12: &[u8] = &[0x26, 0x26, 0x26, 0xBA]; // VGLX: Ratio
                                                            // nt35510_reg10 is missing in HAL
const NT35510_WRITES_13: &[u8] = &[0x00, 0x80, 0x00, 0xBC]; // VGMP/VGSP: 4.5V/0V
const NT35510_WRITES_14: &[u8] = &[0x00, 0x80, 0x00, 0xBD]; // VGMN/VGSN:-4.5V/0V
const NT35510_WRITES_15: &[u8] = &[0x00, 0x50, 0xBE]; // VCOM: -1.325V
const NT35510_WRITES_16: &[u8] = &[0x55, 0xAA, 0x52, 0x08, 0x00, 0xF0]; // LV2: Page 0 enable
const NT35510_WRITES_17: &[u8] = &[0xFC, 0x00, 0xB1]; // Display control
const NT35510_WRITES_18: &[u8] = &[0xB6, 0x03]; // Src hold time
const NT35510_WRITES_19: &[u8] = &[0xB5, 0x51];
const NT35510_WRITES_20: &[u8] = &[0x00, 0x00, 0xB7]; // Gate EQ control
const NT35510_WRITES_21: &[u8] = &[0x01, 0x02, 0x02, 0x02, 0xB8]; // Src EQ control(Mode2)
const NT35510_WRITES_22: &[u8] = &[0x00, 0x00, 0x00, 0xBC]; // Inv. mode(2-dot)
const NT35510_WRITES_23: &[u8] = &[0x03, 0x00, 0x00, 0xCC];
const NT35510_WRITES_24: &[u8] = &[0xBA, 0x01];

const NT35510_MADCTL_PORTRAIT: &[u8] = &[NT35510_CMD_MADCTL, 0x00];
const NT35510_CASET_PORTRAIT: &[u8] = &[0x00, 0x00, 0x01, 0xDF, NT35510_CMD_CASET];
const NT35510_RASET_PORTRAIT: &[u8] = &[0x00, 0x00, 0x03, 0x1F, NT35510_CMD_RASET];
const NT35510_MADCTL_LANDSCAPE: &[u8] = &[NT35510_CMD_MADCTL, 0x60];
const NT35510_CASET_LANDSCAPE: &[u8] = &[0x00, 0x00, 0x03, 0x1F, NT35510_CMD_CASET];
const NT35510_RASET_LANDSCAPE: &[u8] = &[0x00, 0x00, 0x01, 0xDF, NT35510_CMD_RASET];

const NT35510_WRITES_26: &[u8] = &[NT35510_CMD_TEEON, 0x00]; // Tear on
const NT35510_WRITES_27: &[u8] = &[NT35510_CMD_SLPOUT, 0x00]; // Sleep out
                                                              // 28,29 missing
const NT35510_WRITES_30: &[u8] = &[NT35510_CMD_DISPON, 0x00]; // Display on

const NT35510_WRITES_31: &[u8] = &[NT35510_CMD_WRDISBV, 0x7F];
const NT35510_WRITES_32: &[u8] = &[NT35510_CMD_WRCTRLD, 0x2C];
const NT35510_WRITES_33: &[u8] = &[NT35510_CMD_WRCABC, 0x02];
const NT35510_WRITES_34: &[u8] = &[NT35510_CMD_WRCABCMB, 0xFF];
const NT35510_WRITES_35: &[u8] = &[NT35510_CMD_RAMWR, 0x00];

//const NT35510_WRITES_36: &[u8] = &[NT35510_CMD_COLMOD, NT35510_COLMOD_RGB565]; // FIXME: Example sets it to 888 but rest of the code seems to configure DSI for 565
const NT35510_WRITES_37: &[u8] = &[NT35510_CMD_COLMOD, NT35510_COLMOD_RGB888];

// More of these: https://elixir.bootlin.com/linux/latest/source/include/video/mipi_display.h#L83
const NT35510_CMD_TEEON_GET_DISPLAY_ID: u8 = 0x04;

const NT35510_CMD_TEEON: u8 = 0x35;
const NT35510_CMD_MADCTL: u8 = 0x36;

const NT35510_CMD_SLPOUT: u8 = 0x11;
const NT35510_CMD_DISPON: u8 = 0x29;
const NT35510_CMD_CASET: u8 = 0x2A;
const NT35510_CMD_RASET: u8 = 0x2B;
const NT35510_CMD_RAMWR: u8 = 0x2C; /* Memory write */
const NT35510_CMD_COLMOD: u8 = 0x3A;

const NT35510_CMD_WRDISBV: u8 = 0x51; /* Write display brightness */
const NT35510_CMD_RDDISBV: u8 = 0x52; /* Read display brightness */
const NT35510_CMD_WRCTRLD: u8 = 0x53; /* Write CTRL display */
const NT35510_CMD_RDCTRLD: u8 = 0x54; /* Read CTRL display value */
const NT35510_CMD_WRCABC: u8 = 0x55; /* Write content adaptative brightness control */
const NT35510_CMD_WRCABCMB: u8 = 0x5E; /* Write CABC minimum brightness */

const NT35510_COLMOD_RGB565: u8 = 0x55;
const NT35510_COLMOD_RGB888: u8 = 0x77;
