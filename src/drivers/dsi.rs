use embassy_stm32::{
    dsihost::DsiHost,
    pac::{
        dsihost::regs::{Ier0, Ier1},
        DSIHOST,
    },
    peripherals::{DSIHOST, PJ2},
};

use defmt::info;

use crate::config::{Dimensions, Orientation};

/* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
const LANE_BYTE_CLK_K_HZ: u16 = 62500; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L224C21-L224C26

const LCD_CLOCK: u16 = 27429; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L183

/* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
const TXESCAPE_CKDIV: u8 = (LANE_BYTE_CLK_K_HZ / 15620) as u8; // https://github.com/STMicroelectronics/32f469idiscovery-bsp/blob/ec051de2bff3e1b73a9ccd49c9b85abf7320add9/stm32469i_discovery_lcd.c#L230

pub struct Dsi<'a> {
    dsi: DsiHost<'a, DSIHOST>,
}

pub struct Config {
    pub width: u16,
    pub height: u16,

    hact: u16,
    vact: u16,

    pub vsa: u16,
    pub vbp: u16,
    pub vfp: u16,

    pub hsa: u16,
    pub hbp: u16,
    pub hfp: u16,

    virtual_channel_id: u8,

    color_coding: u8,
    pub vs_polarity: bool, // DSI_VSYNC_ACTIVE_HIGH == 0
    pub hs_polarity: bool, // DSI_HSYNC_ACTIVE_HIGH == 0
    pub de_polarity: bool, // DSI_DATA_ENABLE_ACTIVE_HIGH == 0
    mode: u8,              // DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
    null_packet_size: u16,
    number_of_chunks: u16,
    packet_size: u16,
    lp_command_enable: bool, /* Enable sending commands in mode LP (Low Power) */

    /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
    /* Only useful when sending LP packets is allowed while streaming is active in video mode */
    lp_largest_packet_size: u8,

    /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
    /* Only useful when sending LP packets is allowed while streaming is active in video mode */
    lp_vact_largest_packet_size: u8,

    lp_horizontal_front_porch_enable: bool, /* Allow sending LP commands during HFP period */
    lp_horizontal_back_porch_enable: bool,  /* Allow sending LP commands during HBP period */
    lp_vertical_active_enable: bool,        /* Allow sending LP commands during VACT period */
    lp_vertical_front_porch_enable: bool,   /* Allow sending LP commands during VFP period */
    lp_vertical_back_porch_enable: bool,    /* Allow sending LP commands during VBP period */
    lp_vertical_sync_active_enable: bool, /* Allow sending LP commands during VSync = VSA period */
    frame_bta_acknowledge_enable: bool, /* Frame bus-turn-around acknowledge enable => false according to debugger */

    /* Configure DSI PHY HS2LP and LP2HS timings */
    clock_lane_hs2_lp_time: u16,
    clock_lane_lp2_hs_time: u16,
    data_lane_hs2_lp_time: u8,
    data_lane_lp2_hs_time: u8,
    data_lane_max_read_time: u16,
    stop_wait_time: u8,
}

impl Config {
    pub const fn stm32f469_disco(orientation: Orientation) -> Self {
        const LCD_DIMENSIONS: Dimensions = Dimensions::from(800, 480);
        let width = LCD_DIMENSIONS.get_width(orientation);
        let height = LCD_DIMENSIONS.get_height(orientation);

        const DSI_PIXELFORMAT_RGB888: u8 = 0x05;
        const DSI_PIXELFORMAT_ARGB888: u8 = 0x00;

        Self {
            width,
            height,
            hact: width,
            vact: height,

            vsa: 120,
            vbp: 150,
            vfp: 150,

            hsa: 2,
            hbp: 34,
            hfp: 34,

            virtual_channel_id: 0,

            color_coding: DSI_PIXELFORMAT_RGB888,
            vs_polarity: false, // DSI_VSYNC_ACTIVE_HIGH == 0
            hs_polarity: false, // DSI_HSYNC_ACTIVE_HIGH == 0
            de_polarity: false, // DSI_DATA_ENABLE_ACTIVE_HIGH == 0
            mode: 2,            // DSI_VID_MODE_BURST, /* Mode Video burst ie : one LgP per line */
            null_packet_size: 0xFFF,
            number_of_chunks: 0,
            packet_size: width,      //= hact
            lp_command_enable: true, /* Enable sending commands in mode LP (Low Power) */

            /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
            /* Only useful when sending LP packets is allowed while streaming is active in video mode */
            lp_largest_packet_size: 16,

            /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
            /* Only useful when sending LP packets is allowed while streaming is active in video mode */
            lp_vact_largest_packet_size: 0,

            lp_horizontal_front_porch_enable: true, /* Allow sending LP commands during HFP period */
            lp_horizontal_back_porch_enable: true, /* Allow sending LP commands during HBP period */
            lp_vertical_active_enable: true, /* Allow sending LP commands during VACT period */
            lp_vertical_front_porch_enable: true, /* Allow sending LP commands during VFP period */
            lp_vertical_back_porch_enable: true, /* Allow sending LP commands during VBP period */
            lp_vertical_sync_active_enable: true, /* Allow sending LP commands during VSync = VSA period */
            frame_bta_acknowledge_enable: false, /* Frame bus-turn-around acknowledge enable => false according to debugger */

            /* Configure DSI PHY HS2LP and LP2HS timings */
            clock_lane_hs2_lp_time: 35,
            clock_lane_lp2_hs_time: 35,
            data_lane_hs2_lp_time: 35,
            data_lane_lp2_hs_time: 35,
            data_lane_max_read_time: 0,
            stop_wait_time: 10,
        }
    }
}

/// Convert to number of lane byte clock cycles
pub const fn to_clock_cycles(val: u16, lane_byte_clk_k_hz: u16, lcd_clk: u16) -> u16 {
    ((val as u32 * lane_byte_clk_k_hz as u32) / lcd_clk as u32) as u16
}

impl Dsi<'_> {
    pub fn new(dsi: DSIHOST, tepin: PJ2, config: &Config) -> Self {
        let mut dsi = DsiHost::new(dsi, tepin);

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
        DSIHOST.ccr().modify(|w| w.set_txeckdiv(TXESCAPE_CKDIV));

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
        DSIHOST.vmcr().modify(|w| w.set_vmt(config.mode));

        /* Configure the video packet size */
        DSIHOST.vpcr().modify(|w| w.set_vpsize(config.packet_size));

        /* Set the chunks number to be transmitted through the DSI link */
        DSIHOST
            .vccr()
            .modify(|w| w.set_numc(config.number_of_chunks));

        /* Set the size of the null packet */
        DSIHOST
            .vnpcr()
            .modify(|w| w.set_npsize(config.null_packet_size));

        /* Select the virtual channel for the LTDC interface traffic */
        DSIHOST
            .lvcidr()
            .modify(|w| w.set_vcid(config.virtual_channel_id));

        /* Configure the polarity of control signals */
        DSIHOST.lpcr().modify(|w| {
            w.set_dep(config.de_polarity);
            w.set_hsp(config.hs_polarity);
            w.set_vsp(config.vs_polarity);
        });

        /* Select the color coding for the host */
        DSIHOST.lcolcr().modify(|w| w.set_colc(config.color_coding));

        /* Select the color coding for the wrapper */
        DSIHOST
            .wcfgr()
            .modify(|w| w.set_colmux(config.color_coding));

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
            .modify(|w| w.set_hsa(to_clock_cycles(config.hsa, LANE_BYTE_CLK_K_HZ, LCD_CLOCK)));

        /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
        DSIHOST
            .vhbpcr()
            .modify(|w| w.set_hbp(to_clock_cycles(config.hbp, LANE_BYTE_CLK_K_HZ, LCD_CLOCK)));

        /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
        DSIHOST.vlcr().modify(|w| {
            w.set_hline(to_clock_cycles(
                config.hact + config.hsa + config.hbp + config.hfp,
                //VACT + VSA + VBP + VFP,
                LANE_BYTE_CLK_K_HZ,
                LCD_CLOCK,
            ))
        }); /* FIXME: Value depending on display orientation choice portrait/landscape */

        /* Set the Vertical Synchronization Active (VSA) */
        DSIHOST.vvsacr().modify(|w| w.set_vsa(config.vsa));

        /* Set the Vertical Back Porch (VBP)*/
        DSIHOST.vvbpcr().modify(|w| w.set_vbp(config.vbp));

        /* Set the Vertical Front Porch (VFP)*/
        DSIHOST.vvfpcr().modify(|w| w.set_vfp(config.vfp));

        /* Set the Vertical Active period*/
        DSIHOST.vvacr().modify(|w| w.set_va(config.vact));

        /* Configure the command transmission mode */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lpce(config.lp_command_enable));

        /* Low power largest packet size */
        DSIHOST
            .lpmcr()
            .modify(|w| w.set_lpsize(config.lp_largest_packet_size));

        /* Low power VACT largest packet size */
        DSIHOST
            .lpmcr()
            .modify(|w| w.set_vlpsize(config.lp_vact_largest_packet_size));

        /* Enable LP transition in HFP period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lphfpe(config.lp_horizontal_front_porch_enable));

        /* Enable LP transition in HBP period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lphbpe(config.lp_horizontal_back_porch_enable));

        /* Enable LP transition in VACT period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lpvae(config.lp_vertical_active_enable));

        /* Enable LP transition in VFP period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lpvfpe(config.lp_vertical_front_porch_enable));

        /* Enable LP transition in VBP period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lpvbpe(config.lp_vertical_back_porch_enable));

        /* Enable LP transition in vertical sync period */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_lpvsae(config.lp_vertical_sync_active_enable));

        /* Enable the request for an acknowledge response at the end of a frame */
        DSIHOST
            .vmcr()
            .modify(|w| w.set_fbtaae(config.frame_bta_acknowledge_enable));

        /*
        ######################################
        END HAL_DSI_ConfigVideoMode()
        ######################################
        */

        let max_time = core::cmp::max(config.clock_lane_hs2_lp_time, config.clock_lane_lp2_hs_time);

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
            w.set_hs2lp_time(max_time);
            w.set_lp2hs_time(max_time)
        });

        // Data lane timer configuration
        DSIHOST.dltcr().modify(|w| {
            w.set_hs2lp_time(config.data_lane_hs2_lp_time);
            w.set_lp2hs_time(config.data_lane_lp2_hs_time);
            w.set_mrd_time(config.data_lane_max_read_time);
        });

        // Configure the wait period to request HS transmission after a stop state
        DSIHOST
            .pconfr()
            .modify(|w| w.set_sw_time(config.stop_wait_time));

        /*
        ######################################
         END HAL_DSI_ConfigPhyTimer()
        ######################################
        */
        Self { dsi }
    }

    pub fn enable(&mut self) {
        self.dsi.enable();
        self.dsi.enable_wrapper_dsi();
    }

    pub fn write_cmd(
        &mut self,
        channel_id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), embassy_stm32::dsihost::Error> {
        self.dsi.write_cmd(channel_id, address, data)
    }
}
