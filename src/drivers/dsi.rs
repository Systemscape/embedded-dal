
/// Convert to number of lane byte clock cycles
pub const fn to_clock_cycles(val: u16, lane_byte_clk_k_hz: u16, lcd_clk: u16) -> u16 {
    ((val as u32 * lane_byte_clk_k_hz as u32) / lcd_clk as u32) as u16
}
