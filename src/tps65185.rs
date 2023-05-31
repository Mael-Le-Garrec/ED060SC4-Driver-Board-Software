use panic_halt as _;

use embedded_hal::{
    digital::v2::{OutputPin, InputPin},
    blocking::i2c::Write as i2cWrite,
    blocking::delay::{DelayMs, DelayUs},
};
use embedded_error::ImplError::{Internal, self};

/// Starts the PMIC TPS65185 and configures the differents rails.
/// The 3.3 rail is first activated, then the given vcom voltage programmed
/// The rails -20V, -15V, +15V and +22V are then powered up via the default sequence
/// If a problem happend and the rails couldn't be brought up, and Error is returned
/// -  `vcom_voltage`: negative voltage to be achieved by the vcom converter
pub fn configure_TPS<A: i2cWrite, B: OutputPin, C: OutputPin, D: OutputPin, E: InputPin, F: DelayUs<u32>+DelayMs<u32>>(i2c: &mut A, delay: &mut F, wake_up: &mut B, power_up: &mut C, vcom_ctrl: &mut D, vcom_voltage: f32, pwrgood: &E) -> Result<(), ImplError>
{
    // Activate the I2C communication to configure the TPS
    let tps_address = 0x68;
    wake_up.set_high();
    delay.delay_ms(50_u32);  // Give time to the TPS to wakeup

    // Enable 3.3V
    let mut configured = match i2c.write(tps_address, &[0x01, 0x20]) { Ok(_) => true, Err(_) => false };
    delay.delay_us(150_u32);  // min 100µs as per EPD datasheet

    // Enable VCOM (The steps are 10mv, the input is abs(voltage [V] * 100))
    // Most significant bit on VCOM2
    let voltage = (-vcom_voltage * 100.0) as i32;
    let vcom1 = voltage as u8 & 0xFF;
    let vcom2 = (voltage as u32 >> 8) as u8;
    configured = match i2c.write(tps_address, &[0x03, vcom1]) { Ok(_) => configured, Err(_) => false};
    configured = match i2c.write(tps_address, &[0x04, vcom2]) { Ok(_) => configured, Err(_) => false};
    vcom_ctrl.set_high();

    // Start powering up the rails if the configuration was successful
    if configured {
        power_up.set_high();
    }
    else { 
        return Err(Internal);
    }

    // Check from the TPS that all the rails are OK
    if !pwrgood.is_high().unwrap_or(false)
    {
        return Err(Internal);
    }

    Ok(())
}