#![no_std]               // Disable the standard library
#![no_main]              // No main function (kernel entry point)

use core::arch::asm;      // Import inline assembly support
use core::panic::PanicInfo;

// Base address for GPIO
const GPIO_BASE: u32 = 0x20200000;

// GPIO Function Select Registers
const GPFSEL3: u32 = GPIO_BASE + 0x0C;  // GPIO Function Select 3 (for pins 10-19)
const GPFSEL4: u32 = GPIO_BASE + 0x10;  // GPIO Function Select 4 (for pins 20-29)

// GPIO Pin Output Set/Clear Registers
const GPSET0: u32 = GPIO_BASE + 0x1C;  // GPIO Pin Set 0 (for pins 0-31)
const GPCLR0: u32 = GPIO_BASE + 0x28;  // GPIO Pin Clear 0 (for pins 0-31)
const GPSET1: u32 = GPIO_BASE + 0x20;  // GPIO Pin Set 1 (for pins 32-53)
const GPCLR1: u32 = GPIO_BASE + 0x2C;  // GPIO Pin Clear 1 (for pins 32-53)

// GPIO Pins
const GPIO_PWR_PIN: u32 = 47;  // GPIO47 for PWR LED (in second bank, 32-53)
const GPIO_ACT_PIN: u32 = 35;  // GPIO35 for ACT LED (in first bank, 0-31)

#[no_mangle]
pub extern "C" fn _start() -> ! {
    unsafe {
        // Set GPIO47 (PWR LED) as output (in GPFSEL4, bits 21-23)
        let fsel4 = GPFSEL4 as *mut u32;
        let current = fsel4.read_volatile();
        fsel4.write_volatile((current & !(0b111 << 21)) | (0b001 << 21)); // Set GPIO47 to output

        // Set GPIO35 (ACT LED) as output (in GPFSEL3, bits 15-17)
        let fsel3 = GPFSEL3 as *mut u32;
        let current = fsel3.read_volatile();
        fsel3.write_volatile((current & !(0b111 << 15)) | (0b001 << 15)); // Set GPIO35 to output

        // Infinite loop to toggle the LEDs
        loop {
            // Set GPIO47 (PWR LED) HIGH (turn on)
            if GPIO_PWR_PIN >= 32 {
                // GPIO47 is in the second bank, so we need to shift (47 - 32)
                let set1 = GPSET1 as *mut u32;
                set1.write_volatile(1 << (GPIO_PWR_PIN - 32)); // Set GPIO 47 (PWR LED)
            }

            // Set GPIO35 LOW (turn off ACT LED)
            let clr0 = GPCLR0 as *mut u32;
            clr0.write_volatile(1 << GPIO_ACT_PIN); // Clear GPIO35 (ACT LED OFF)
            
            // Delay
            delay();

            // Set GPIO47 LOW (turn off PWR LED)
            if GPIO_PWR_PIN >= 32 {
                let clr1 = GPCLR1 as *mut u32;
                clr1.write_volatile(1 << (GPIO_PWR_PIN - 32)); // Clear GPIO 47 (PWR LED OFF)
            }

            // Set GPIO35 HIGH (turn on ACT LED)
            let set0 = GPSET0 as *mut u32;
            set0.write_volatile(1 << GPIO_ACT_PIN); // Set GPIO35 (ACT LED ON)

            // Delay
            delay();
        }
    }
}

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    // Infinite loop on panic to halt execution
    loop {
        unsafe { asm!("nop"); } // No operation (just idling)
    }
}

// Simple delay function
fn delay() {
    for _ in 0..0x100000 {
        // Just a simple busy loop for delay
        unsafe { asm!("nop"); }
    }
}
