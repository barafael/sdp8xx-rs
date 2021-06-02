use linux_embedded_hal::{Delay, I2cdev};
use sdp8xx::Sdp8xx;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x25;
    let mut sdp = Sdp8xx::new(dev, address, Delay);

    println!("Starting Sdp8xx tests.");

    if let Ok(product_id) = sdp.read_product_id() {
        println!("{:?}", product_id);
    } else {
        println!("Could not read product ID.");
    }

    println!("Taking 10 triggered samples");
    for _ in 0..=10 {
        if let Ok(m) = sdp.trigger_differential_pressure_sample() {
            println!("{:?}", m);
        } else {
            println!("Error!");
        }
    }

    println!("Going to sleep!");
    let sleeping = match sdp.go_to_sleep() {
        Ok(sdp) => { sdp }
        Err(_) => {
            println!("Could not go to sleep");
            loop {}
        }
    };

    println!("Sleeping.");
    std::thread::sleep(std::time::Duration::from_millis(500));

    let sdp = match sleeping.wake_up() {
        Ok(woke) => woke,
        Err(_) => {
            println!("Could not wake up sensor.");
            loop {}
        }
    };
    println!("Woken up!");

    let mut sdp_sampling = match sdp.start_sampling_differential_pressure(true) {
        Ok(s) => s,
        Err(_) => {
            println!("Could not start sampling.");
            loop {}
        }
    };
    std::thread::sleep(std::time::Duration::from_millis(100));
    println!("Starting to take all the samples");

    for _ in 0..=50 {
        let result = sdp_sampling.read_continuous_sample();
        match result {
            Ok(r) => println!("{:?}", r),
            Err(_) => println!("Error while getting result."),
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    let mut idle_sensor = match sdp_sampling.stop_sampling() {
        Ok(s) => s,
        Err(_) => {
            println!("Couldn't stop sampling.");
            loop {}
        }
    };
    loop {
        if let Ok(m) = idle_sensor.trigger_differential_pressure_sample() {
            println!("{:?}", m);
        } else {
            println!("Error!");
        }
    }
}
