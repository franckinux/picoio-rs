#![no_std]
#![no_main]

// use defmt::*;
// use defmt_rtt as _;
use embedded_hal::{
    adc::OneShot,
    digital::v2::{
        InputPin,
        OutputPin,
    },
    PwmPin,
};
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

use hal::{
    adc::Adc,
    adc::AdcPin,
    gpio::{
        DynPinId,
        FunctionSioInput,
        FunctionSioOutput,
        Pin,
        Pins,
        PullNone,
    },
    pac,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
    usb::UsbBus,
};
use crc::{Crc, CRC_16_MODBUS};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const GPIO_ADC_START: usize = 26;
const GPIO_ADC_NUMBER: usize = 3;
const GPIO_GDI_START: usize = 12;
const GPIO_GDI_NUMBER: usize = 6;
const GPIO_PWM_START: usize = 18;
const GPIO_PWM_NUMBER: usize = 4;
const GPIO_SDO_START: usize = 0;
const GPIO_SDO_NUMBER: usize = 12;

const ERROR_UNKNOWN_COMMAND: u16 = 1;
const ERROR_INVALID_LENGTH: u16 = 2;
const ERROR_INVALID_FORMAT: u16 = 3;
const ERROR_INVALID_CRC16: u16 = 4;
const ERROR_INVALID_VALUE: u16 = 5;

const PWM_COUNTER: u16 =  62500;


fn ascii2dec(buf: &[u8]) -> u16 {
    let mut v: u16 = 0;
    for &c in buf.iter() {
        if c == b',' || c == b'$' {
            break;
        }
        v = v*10 + c as u16 - b'0' as u16;
    }
    v
}


fn ascii2bin(buf: &[u8]) -> u16 {
    let mut v: u16 = 0;
    for &c in buf.iter() {
        if c == b',' || c == b'$' {
            break;
        }

        v = (v << 4) + c as u16;
        if c <= b'9' {
            v -= b'0' as u16;
        } else {
            if c <= b'F' {
                v -= b'A' as u16;
            } else {
                if c <= b'f' {
                    v -= b'a' as u16;
                }
            }
            v += 10;
        }
    }
    v
}


fn crc_modbus(buffer: &[u8]) -> u16 {
    let crc = Crc::<u16>::new(&CRC_16_MODBUS);
	let mut digest = crc.digest();
	digest.update(buffer);
    digest.finalize()
}


fn data(ser: &mut SerialPort<UsbBus>, v: u16) {
    let mut buf1 = [0u8; 16];
    let mut buf2 = [0u8; 16];

    let s = format_no_std::show(
        &mut buf1,
        format_args!("^DATA {:04}", v),
    ).unwrap();
    let c = crc_modbus(&s[1..].as_bytes());
    let s = format_no_std::show(
        &mut buf2,
        format_args!("{},{:04X}$", s, c),
    ).unwrap();
    ser.write(&s[..].as_bytes()).unwrap();
}


fn ok(ser: &mut SerialPort<UsbBus>) {
    let mut buf2 = [0u8; 16];

    let buf1 = "^OK ";
    let c = crc_modbus(&buf1[1..].as_bytes());
    let s = format_no_std::show(
        &mut buf2,
        format_args!("{}{:04X}$", buf1, c),
    ).unwrap();
    ser.write(&s[..].as_bytes()).unwrap();
}


fn nok(ser: &mut SerialPort<UsbBus>, err: u16) {
    let mut buf1 = [0u8; 16];
    let mut buf2 = [0u8; 16];

    let s = format_no_std::show(
        &mut buf1,
        format_args!("^NOK {:04}", err),
    ).unwrap();
    let c = crc_modbus(&s[1..].as_bytes());
    let s = format_no_std::show(
        &mut buf2,
        format_args!("{},{:04X}$", s, c),
    ).unwrap();
    ser.write(&s[..].as_bytes()).unwrap();
}


/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ) .ok().unwrap();

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x55aa, 0x0001))
        .manufacturer("Fake company")
        .product("picoio-rs")
        .serial_number("00000001")
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // configure outputs/input/adc/pwm
    let mut output_pins: [Pin<DynPinId, FunctionSioOutput, PullNone>; 12] = [
        pins.gpio0
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio1
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio2
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio3
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio4
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio5
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio6
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio7
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio8
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio9
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio10
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio11
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
    ];

    let input_pins: [Pin<DynPinId, FunctionSioInput, PullNone>; 6] = [
        pins.gpio12
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio13
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio14
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio15
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio16
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio17
            .into_pull_up_input()
            .into_pull_type()
            .into_dyn_pin(),
    ];

    let mut adc_pin_0 = AdcPin::new(pins.gpio26);
    let mut adc_pin_1 = AdcPin::new(pins.gpio27);
    let mut adc_pin_2 = AdcPin::new(pins.gpio28);

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut led_state = false;

    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM slice 1 channels A & B
    let mut pwm1 = pwm_slices.pwm1;
    pwm1.set_ph_correct();
    pwm1.set_top(PWM_COUNTER);
    pwm1.set_div_int(20);  // 125MHz/20/62500 = 100Hz
    pwm1.set_div_frac(0);
    pwm1.enable();

    // Configure PWM slice 2 channel A
    let mut pwm2 = pwm_slices.pwm2;
    pwm2.set_ph_correct();
    pwm2.set_top(PWM_COUNTER);
    pwm2.set_div_int(20);  // 125MHz/20/62500 = 100Hz
    pwm2.set_div_frac(0);
    pwm2.enable();

    // Output channel A on PWM1 to GPIO 18
    let channel_pin_1a = &mut pwm1.channel_a;
    channel_pin_1a.output_to(pins.gpio18);

    // Output channel B on PWM1 to GPIO 19
    let channel_pin_1b = &mut pwm1.channel_b;
    channel_pin_1b.output_to(pins.gpio19);

    // Output channel A on PWM2 to GPIO 20
    let channel_pin_2a = &mut pwm2.channel_a;
    channel_pin_2a.output_to(pins.gpio20);

    // Output channel B on PWM2 to GPIO 21
    let channel_pin_2b = &mut pwm2.channel_b;
    channel_pin_2b.output_to(pins.gpio21);

    // set default outputs
    channel_pin_1a.set_duty(0);
    channel_pin_1b.set_duty(0);
    channel_pin_2a.set_duty(0);

    let mut buffer = [0; 64];
    loop {
        // delay.delay_ms(10);
        if !usb_dev.poll(&mut [&mut serial]) {
            continue
        }

        if let Ok(count) = serial.read(&mut buffer[..]) {
            if buffer[0] != b'^' {
                continue;
            }
            if &buffer[1..5] == b"SDO " {
                if count == 15 {
                    if buffer[7] == b',' && buffer[9] == b',' {
                        let n = ascii2dec(&buffer[5..7]) as usize;
                        let v = ascii2dec(&buffer[8..9]);
                        if n >= GPIO_SDO_START && n < GPIO_SDO_START + GPIO_SDO_NUMBER
                            && (v == 0 || v == 1) {
                            let c = ascii2bin(&buffer[10..14]);
                            let c2 = crc_modbus(&buffer[1..9]);
                            if c == c2 {
                                if v == 0 {
                                    output_pins[n].set_low().unwrap();
                                } else {
                                    output_pins[n].set_high().unwrap();
                                }
                                ok(&mut serial);
                            } else {
                                nok(&mut serial, ERROR_INVALID_CRC16);
                            }
                        } else {
                            nok(&mut serial, ERROR_INVALID_VALUE);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else if &buffer[1..6] == b"SDOS " {
                if count == 21 {
                    if buffer[10] == b',' && buffer[15] == b',' {
                        let mut v = ascii2dec(&buffer[6..10]);
                        let mut m = ascii2dec(&buffer[11..12]);
                        let c = ascii2bin(&buffer[16..20]);
                        let c2 = crc_modbus(&buffer[1..15]);
                        if c == c2 {
                            for i in 0..GPIO_SDO_NUMBER {
                                if v & m == 0 {
                                    output_pins[i].set_low().unwrap();
                                } else {
                                    output_pins[i].set_high().unwrap();
                                }
                                v >>= 1;
                                m >>= 1;
                            }
                            ok(&mut serial);
                        } else {
                            nok(&mut serial, ERROR_INVALID_CRC16);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else if &buffer[1..5] == b"GDI " {
                if count == 13 {
                    if buffer[7] == b',' {
                        let n = ascii2dec(&buffer[8..12]) as usize;
                        if n >= GPIO_GDI_START && n < GPIO_GDI_START + GPIO_GDI_NUMBER {
                            let c = ascii2bin(&buffer[8..12]);
                            let c2 = crc_modbus(&buffer[1..7]);
                            if c == c2 {
                                let v = if input_pins[n].is_high().unwrap() {1} else {0};
                                data(&mut serial, v);
                            } else {
                                nok(&mut serial, ERROR_INVALID_CRC16);
                            }
                        } else {
                            nok(&mut serial, ERROR_INVALID_VALUE);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else if &buffer[1..6] == b"GDIS " {
                if count == 13 {
                    if buffer[7] == b',' {
                        let c = ascii2bin(&buffer[8..12]);
                        let c2 = crc_modbus(&buffer[1..7]);
                        if c == c2 {
                            let mut v = 0;
                            for i in (0..GPIO_GDI_NUMBER).rev() {
                                v <<= 1;
                                if input_pins[i].is_high().unwrap() {
                                    v |= 1;
                                }
                            }
                            data(&mut serial, v);
                        } else {
                            nok(&mut serial, ERROR_INVALID_CRC16);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else if &buffer[1..5] == b"PWM " {
                if count == 18 {
                    if buffer[7] == b',' && buffer[12] == b',' {
                        let n = ascii2dec(&buffer[5..7]) as usize;
                        let v = ascii2dec(&buffer[8..12]);
                        if n >= GPIO_PWM_START && n < GPIO_PWM_START + GPIO_PWM_NUMBER {
                            let c = ascii2bin(&buffer[13..17]);
                            let c2 = crc_modbus(&buffer[1..12]);
                            if c == c2 {
                                match n {
                                    18 => channel_pin_1a.set_duty(v * PWM_COUNTER / 10000),
                                    19 => channel_pin_1b.set_duty(v * PWM_COUNTER / 10000),
                                    20 => channel_pin_2a.set_duty(v * PWM_COUNTER / 10000),
                                    21 => channel_pin_2b.set_duty(v * PWM_COUNTER / 10000),
                                    _ => panic!(),  // not reachable !
                                };
                                ok(&mut serial);
                            } else {
                                nok(&mut serial, ERROR_INVALID_CRC16);
                            }
                        } else {
                            nok(&mut serial, ERROR_INVALID_VALUE);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else if &buffer[1..5] == b"ADC " {
                if count == 13 {
                    if buffer[7] == b',' {
                        let n = ascii2dec(&buffer[8..12]) as usize;
                        if n >= GPIO_ADC_START && n < GPIO_ADC_START + GPIO_ADC_NUMBER {
                            let c = ascii2bin(&buffer[8..12]);
                            let c2 = crc_modbus(&buffer[1..7]);
                            if c == c2 {
                                let v = match n {
                                    26 => adc.read(&mut adc_pin_0).unwrap(),
                                    27 => adc.read(&mut adc_pin_1).unwrap(),
                                    28 => adc.read(&mut adc_pin_2).unwrap(),
                                    _ => panic!(),  // not reachable !
                                };
                                data(&mut serial, v);
                            } else {
                                nok(&mut serial, ERROR_INVALID_CRC16);
                            }
                        } else {
                            nok(&mut serial, ERROR_INVALID_VALUE);
                        }
                    } else {
                        nok(&mut serial, ERROR_INVALID_FORMAT);
                    }
                } else {
                    nok(&mut serial, ERROR_INVALID_LENGTH);
                }
            } else {
                nok(&mut serial, ERROR_UNKNOWN_COMMAND);
            }

            if led_state {
                led_pin.set_low().unwrap();
            } else {
                led_pin.set_high().unwrap();
            }
            led_state = !led_state;
        }

    }
}

// End of file
