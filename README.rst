Derived from https://github.com/aptlyundecided/super-blank-rust-pico-project.git

## Install

cargo install elf2uf2-rs --locked
rustup target add thumbv6m-none-eabi

## USB examples

https://github.com/joaocarvalhoopen/Raspberry_Pi_Pico_in_Rust__Proj_Template_with_RTIC_USB-Serial_UF2
https://docs.rs/usbd-serial/0.1.1/usbd_serial/
https://docs.rs/rp2040-hal/latest/rp2040_hal/usb/index.html

## License
The contents of this repository are dual-licensed under the MIT OR
Apache 2.0 License. That means you can choose either the MIT license
or the Apache-2.0 license when you re-use this code. See MIT or APACHE2.0
for more information on each specific license.



Reference
---------

https://www.gibbard.me/using_the_raspberry_pi_pico_on_ubuntu/

Install SDK
-----------

sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential libstdc++-arm-none-eabi-newlib
git submodule add -b master https://github.com/raspberrypi/pico-sdk.gi
cd pico-sdk/
git submodule update --init
cd ..
export PICO_SDK_PATH=~/Documents/repositories/geneseas/picoio/pico-sdk

See examples
------------

git submodule add export PICO_SDK_PATH=~/Documents/repositories/geneseas/picoio/pico-sdk

Create own project
------------------

in directory picoio

cd picoio/build
cmake -GNinja ..
ninja

press BOOTSEL button
insert in usb connector
release BOOTSEL button

cp picoio.uf2 /media/franck/RPI-RP2

that's all!


Serial protocol
---------------

Checksum is a crc16. Start '^' and end '$' characters are excluded.

Set PWM pulse width
...................

Command :

    ^PWM nn,xxxx,zzzz$

    where
        - nn: channel number in 18 - 22
        - xxxx: pulse width duration. decimal number from 1000 to 2000 or 0
        - zzzz: crc in hex

Response :

    ^OK zzzz$
    ^NOK wwww,zzzz$

    where
        - wwww: error code
        - zzzz: crc in hex

Read ADC value
..............

Command :

    ^ADC nn,zzzz$

    where
        - nn: channel number in 26 - 29
        - zzzz: crc in hex

Response :

    ^DATA xxxx,zzzz$
    ^NOK wwww,zzzz$

    where
        - xxxx: raw value in hex
        - wwww: error code
        - zzzz: crc in hex

Set digital output
................

Command :

    ^SDO nn,x,zzzz$

    where
        - n: digital out number in 0 -  11
        - x: logical value in 0, 1
        - zzzz: crc in hex

Response :

    ^OK zzzz$
    ^NOK wwww,zzzz$

    where
        - wwww: error code
        - zzzz: crc in hex

Set all digital outputs
.......................

Command :

    ^SDOS xxxx,yyyy,zzzz$

    where
        - xxxx: logical values in hex
        - yyyy: mask value in hex
        - zzzz: crc in hex

Response :

    ^OK zzzz$
    ^NOK wwww,zzzz$

    where
        - wwww: error code
        - zzzz: crc in hex

Get digital input
.................

Command :

    ^GDI nn,zzzz$

    where
       - nn: digital out number in 12 - 17
       - zzzz: crc in hex

Response :

    ^DATA xxxx,zzzz$
    ^NOK wwww,zzzz$

    where
        - xxxx: logical value in 0, 1
        - wwww: error code
        - zzzz: crc in hex


Get all digital inputs
......................

Command :

    ^GDIS _,zzzz$

    where
       - zzzz: crc in hex

Response :

    ^DATA xxxx,zzzz$
    ^NOK wwww,zzzz$

    where
        - xxxx: logical value in hex
        - wwww: error code
        - zzzz: crc in hex


 01234567890123
^ADC nn,zzzz$
^GDI nn,zzzz$
^GDIS _,zzzz$
^PWM nn,xxxx,zzzz$
^SDO nn,x,zzzz$
^SDOS xxxx,yyyy,zzzz$
