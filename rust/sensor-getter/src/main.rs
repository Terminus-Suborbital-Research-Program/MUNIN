pub mod data_packet;
pub mod server;

use bmm350;
use bmi323;
use std::error::Error;
use std::thread;
use std::time::Duration;
use rppal::gpio::Gpio;

const LED_PIN: u8 = 14;
fn main() {
    let mut led_blink = Gpio::new().unwrap().get(LED_PIN).unwrap().into_output();

    loop {
        led_blink.toggle();
        println!("wow");        
        thread::sleep(Duration::from_millis(1500));
    }
    
}
