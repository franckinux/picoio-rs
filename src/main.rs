//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

// use defmt::*;
// use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use rp_pico as bsp;
use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        watchdog::Watchdog,
        usb::UsbBus,
    },
    Pins,
};
use crc::{Crc, CRC_16_MODBUS};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};


const GPIO_ADC_START: u16 = 26;
const GPIO_ADC_NUMBER: u16 = 4;
const GPIO_GDI_START: u16 = 12;
const GPIO_GDI_NUMBER: u16 = 6;
const GPIO_PWM_START: u16 = 18;
const GPIO_PWM_NUMBER: u16 = 4;
const GPIO_SDO_START: u16 = 0;
const GPIO_SDO_NUMBER: u16 = 12;

const ERROR_UNKNOWN_COMMAND: u16 = 1;
const ERROR_INVALID_LENGTH: u16 = 2;
const ERROR_INVALID_FORMAT: u16 = 3;
const ERROR_INVALID_CRC16: u16 = 4;
const ERROR_INVALID_VALUE: u16 = 5;


// fn write_serial(ser: &mut SerialPort<UsbBus>, string: &str)  {
//     match ser.write(string.as_bytes()) {
//         Ok(_count) => {
//             // count bytes were written
//         },
//         Err(UsbError::WouldBlock) => {},  // No data could be written (buffers full)
//         Err(_err) => {},  // An error occurred
//     };
// }



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


fn execute_sdo(n: u16, v: u16) {
}


fn ok(ser: &mut SerialPort<UsbBus>) {
    let mut buf2 = [0u8; 16];

    let buf1 = "^OK ";
    let c = crc_modbus(&buf1[1..4].as_bytes());
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
        format_args!("^NOK {:04},", err),
    ).unwrap();
    let c = crc_modbus(&s[1..9].as_bytes());
    let s = format_no_std::show(
        &mut buf2,
        format_args!("{}{:04X}$", s, c),
    ).unwrap();
    ser.write(&s[..].as_bytes()).unwrap();
}


#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();
    let mut led_state = false;

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
                        let n = ascii2dec(&buffer[5..7]);
                        let v = ascii2dec(&buffer[8..9]);
                        if n >= GPIO_SDO_START && n < GPIO_SDO_START + GPIO_SDO_NUMBER
                            && (v == 0 || v == 1) {
                            let c = ascii2bin(&buffer[10..14]);
                            let c2 = crc_modbus(&buffer[1..9]);
                            if c == c2 {
                                execute_sdo(n, v);
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
            }

            // serial.write(&buffer[..count]).unwrap();

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
