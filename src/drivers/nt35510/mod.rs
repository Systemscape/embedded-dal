
pub const NT35510_WRITES_0: &[u8] = &[0x55, 0xAA, 0x52, 0x08, 0x01, 0xF0]; // LV2:  Page 1 enable
pub const NT35510_WRITES_1: &[u8] = &[0x03, 0x03, 0x03, 0xB0]; // AVDD: 5.2V
pub const NT35510_WRITES_2: &[u8] = &[0x46, 0x46, 0x46, 0xB6]; // AVDD: Ratio
pub const NT35510_WRITES_3: &[u8] = &[0x03, 0x03, 0x03, 0xB1]; // AVEE: -5.2V
pub const NT35510_WRITES_4: &[u8] = &[0x36, 0x36, 0x36, 0xB7]; // AVEE: Ratio
pub const NT35510_WRITES_5: &[u8] = &[0x00, 0x00, 0x02, 0xB2]; // VCL: -2.5V
pub const NT35510_WRITES_6: &[u8] = &[0x26, 0x26, 0x26, 0xB8]; // VCL: Ratio
pub const NT35510_WRITES_7: &[u8] = &[0xBF, 0x01]; // VGH: 15V (Free Pump)
pub const NT35510_WRITES_8: &[u8] = &[0x09, 0x09, 0x09, 0xB3];
pub const NT35510_WRITES_9: &[u8] = &[0x36, 0x36, 0x36, 0xB9]; // VGH: Ratio
pub const NT35510_WRITES_10: &[u8] = &[0x08, 0x08, 0x08, 0xB5]; // VGL_REG: -10V
pub const NT35510_WRITES_12: &[u8] = &[0x26, 0x26, 0x26, 0xBA]; // VGLX: Ratio
                                                            // nt35510_reg10 is missing in HAL
pub const NT35510_WRITES_13: &[u8] = &[0x00, 0x80, 0x00, 0xBC]; // VGMP/VGSP: 4.5V/0V
pub const NT35510_WRITES_14: &[u8] = &[0x00, 0x80, 0x00, 0xBD]; // VGMN/VGSN:-4.5V/0V
pub const NT35510_WRITES_15: &[u8] = &[0x00, 0x50, 0xBE]; // VCOM: -1.325V
pub const NT35510_WRITES_16: &[u8] = &[0x55, 0xAA, 0x52, 0x08, 0x00, 0xF0]; // LV2: Page 0 enable
pub const NT35510_WRITES_17: &[u8] = &[0xFC, 0x00, 0xB1]; // Display control
pub const NT35510_WRITES_18: &[u8] = &[0xB6, 0x03]; // Src hold time
pub const NT35510_WRITES_19: &[u8] = &[0xB5, 0x51];
pub const NT35510_WRITES_20: &[u8] = &[0x00, 0x00, 0xB7]; // Gate EQ control
pub const NT35510_WRITES_21: &[u8] = &[0x01, 0x02, 0x02, 0x02, 0xB8]; // Src EQ control(Mode2)
pub const NT35510_WRITES_22: &[u8] = &[0x00, 0x00, 0x00, 0xBC]; // Inv. mode(2-dot)
pub const NT35510_WRITES_23: &[u8] = &[0x03, 0x00, 0x00, 0xCC];
pub const NT35510_WRITES_24: &[u8] = &[0xBA, 0x01];

pub const NT35510_MADCTL_PORTRAIT: &[u8] = &[NT35510_CMD_MADCTL, 0x00];
pub const NT35510_CASET_PORTRAIT: &[u8] = &[0x00, 0x00, 0x01, 0xDF, NT35510_CMD_CASET];
pub const NT35510_RASET_PORTRAIT: &[u8] = &[0x00, 0x00, 0x03, 0x1F, NT35510_CMD_RASET];
pub const NT35510_MADCTL_LANDSCAPE: &[u8] = &[NT35510_CMD_MADCTL, 0x60];
pub const NT35510_CASET_LANDSCAPE: &[u8] = &[0x00, 0x00, 0x03, 0x1F, NT35510_CMD_CASET];
pub const NT35510_RASET_LANDSCAPE: &[u8] = &[0x00, 0x00, 0x01, 0xDF, NT35510_CMD_RASET];

pub const NT35510_WRITES_26: &[u8] = &[NT35510_CMD_TEEON, 0x00]; // Tear on
pub const NT35510_WRITES_27: &[u8] = &[NT35510_CMD_SLPOUT, 0x00]; // Sleep out
                                                              // 28,29 missing
pub const NT35510_WRITES_30: &[u8] = &[NT35510_CMD_DISPON, 0x00]; // Display on

pub const NT35510_WRITES_31: &[u8] = &[NT35510_CMD_WRDISBV, 0x7F];
pub const NT35510_WRITES_32: &[u8] = &[NT35510_CMD_WRCTRLD, 0x2C];
pub const NT35510_WRITES_33: &[u8] = &[NT35510_CMD_WRCABC, 0x02];
pub const NT35510_WRITES_34: &[u8] = &[NT35510_CMD_WRCABCMB, 0xFF];
pub const NT35510_WRITES_35: &[u8] = &[NT35510_CMD_RAMWR, 0x00];

//pub const NT35510_WRITES_36: &[u8] = &[NT35510_CMD_COLMOD, NT35510_COLMOD_RGB565]; // FIXME: Example sets it to 888 but rest of the code seems to configure DSI for 565
pub const NT35510_WRITES_37: &[u8] = &[NT35510_CMD_COLMOD, NT35510_COLMOD_RGB888];

// More of these: https://elixir.bootlin.com/linux/latest/source/include/video/mipi_display.h#L83
pub const NT35510_CMD_TEEON_GET_DISPLAY_ID: u8 = 0x04;

pub const NT35510_CMD_TEEON: u8 = 0x35;
pub const NT35510_CMD_MADCTL: u8 = 0x36;

pub const NT35510_CMD_SLPOUT: u8 = 0x11;
pub const NT35510_CMD_DISPON: u8 = 0x29;
pub const NT35510_CMD_CASET: u8 = 0x2A;
pub const NT35510_CMD_RASET: u8 = 0x2B;
pub const NT35510_CMD_RAMWR: u8 = 0x2C; /* Memory write */
pub const NT35510_CMD_COLMOD: u8 = 0x3A;

pub const NT35510_CMD_WRDISBV: u8 = 0x51; /* Write display brightness */
pub const NT35510_CMD_RDDISBV: u8 = 0x52; /* Read display brightness */
pub const NT35510_CMD_WRCTRLD: u8 = 0x53; /* Write CTRL display */
pub const NT35510_CMD_RDCTRLD: u8 = 0x54; /* Read CTRL display value */
pub const NT35510_CMD_WRCABC: u8 = 0x55; /* Write content adaptative brightness control */
pub const NT35510_CMD_WRCABCMB: u8 = 0x5E; /* Write CABC minimum brightness */

pub const NT35510_COLMOD_RGB565: u8 = 0x55;
pub const NT35510_COLMOD_RGB888: u8 = 0x77;