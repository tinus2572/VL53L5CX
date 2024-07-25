#![no_std]
#![no_main]

use vl53l5cx::{
    Vl53l5cx, 
    ResultsData
};

use panic_halt as _; 
use cortex_m_rt::entry;

use core::{fmt::Write, cell::RefCell};

use embedded_hal::i2c::SevenBitAddress;

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PinState::High,
        gpioa, 
        gpiob,
        gpioc,
        Alternate}, 
    pac::{USART2, Peripherals, CorePeripherals, TIM1, TIM2, TIM3}, 
    prelude::*, 
    serial::{Config, Tx}, 
    timer::{Delay, SysDelay},
    rcc::{Rcc, Clocks}
};

// I2C related imports
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{I2c as StmI2c, I2c1, Mode}};
use embedded_hal_bus::i2c::RefCellDevice;

fn write_results_multi(tx: &mut Tx<USART2>, results_top: &ResultsData, results_left: &ResultsData, results_right: &ResultsData, width: usize) {
    writeln!(tx, "\x1B[2H").unwrap();

    writeln!(tx, "VL53L5A1 Multi Sensor demo application\n").unwrap();
    writeln!(tx, "Cell Format :\n").unwrap();
    writeln!(
        tx, 
        "\x1b[96m{dis:>20}\x1b[0m \x1b[92m{sta:<20}\x1b[0m", 
        dis="Distance [mm]", 
        sta="Status"
    ).unwrap();
    writeln!(
        tx, 
        "\x1b[93m{sig:>20}\x1b[0m \x1b[91m{amb:<20}\x1b[0m", 
        sig="Signal [kcps/spad]", 
        amb="Ambient [kcps/spad]"
    ).unwrap();

    writeln!(tx, "\nTop :").unwrap();
    write_results(tx, results_top, width);
    
    writeln!(tx, "\nLeft :").unwrap();
    write_results(tx, results_left, width);
    
    writeln!(tx, "\nRight :").unwrap();
    write_results(tx, results_right, width);
}

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {
    for j in 0..width {
        for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();
        
        #[cfg(not(any(feature="VL53L5CX_DISABLE_DISTANCE_MM", feature="VL53L5CX_DISABLE_TARGET_STATUS")))]
        {
            for i in 0..width {
                write!(
                    tx, 
                    "|\x1b[96m{dis:>5}\x1b[0m \x1b[92m{sta:<4}\x1b[0m", 
                dis=results.distance_mm[width*j+i], 
                sta=results.target_status[width*j+i]
                ).unwrap();
            } write!(tx, "|\n").unwrap();
        }

        #[cfg(not(any(feature="VL53L5CX_DISABLE_SIGNAL_PER_SPAD", feature="VL53L5CX_DISABLE_AMBIENT_PER_SPAD")))]
        {
            for i in 0..width {
                let mut sig: u32 = results.signal_per_spad[width*j+i];
                if sig > 9999 { sig = 9999; }
                write!(
                    tx, 
                    "|\x1b[93m{sig:>5}\x1b[0m \x1b[91m{amb:<4}\x1b[0m", 
                    sig=sig, 
                    amb=results.ambient_per_spad[width*j+i]
                ).unwrap();
            } write!(tx, "|\n").unwrap();
        }
    }
    for _ in 0..width { write!(tx, "+----------").unwrap(); } writeln!(tx, "+").unwrap();

}

const WIDTH: usize = 4;

#[entry]
fn main() -> ! {
    let mut results_top: ResultsData;
    let mut results_left: ResultsData;
    let mut results_right: ResultsData;
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    let _delay: SysDelay = cp.SYST.delay(&clocks);
    let tim_top: Delay<TIM1, 1000> = dp.TIM1.delay_ms(&clocks);
    let tim_left: Delay<TIM2, 1000> = dp.TIM2.delay_ms(&clocks);
    let tim_right: Delay<TIM3, 1000> = dp.TIM3.delay_ms(&clocks);

    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    let gpioc: gpioc::Parts = dp.GPIOC.split();
    
    let _pwr_pin_top: Pin<'B', 0, Output> = gpiob.pb0.into_push_pull_output_in_state(High);
    let _pwr_pin_left: Pin<'A', 0, Output> = gpioa.pa0.into_push_pull_output_in_state(High);
    let _pwr_pin_right: Pin<'C', 0, Output> = gpioc.pc0.into_push_pull_output_in_state(High);

    let lpn_pin_top: Pin<'B', 4, Output> = gpiob.pb4.into_push_pull_output_in_state(High);
    let lpn_pin_left: Pin<'B', 10, Output> = gpiob.pb10.into_push_pull_output_in_state(High);
    let lpn_pin_right: Pin<'B', 5, Output> = gpiob.pb5.into_push_pull_output_in_state(High);
    
    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
     
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(460800.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap();
    
    let _resolution: u8 = (WIDTH * WIDTH) as u8;

    
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    
    let i2c: StmI2c<I2C1> = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard{frequency:200.kHz()},
        &clocks);
        
    let i2c_bus: RefCell<StmI2c<I2C1>> = RefCell::new(i2c);
    let address_top: SevenBitAddress = 0x10;
    let address_left: SevenBitAddress = 0x11;
    let address_right: SevenBitAddress = 0x12;

    let i2c_top = RefCellDevice::new(&i2c_bus);
    let i2c_left = RefCellDevice::new(&i2c_bus);
    let i2c_right = RefCellDevice::new(&i2c_bus);
    
    let mut sensor_top = Vl53l5cx::new_i2c(
        i2c_top, 
        lpn_pin_top,
        tim_top).unwrap();
    let mut sensor_left = Vl53l5cx::new_i2c(
        i2c_left, 
        lpn_pin_left,
        tim_left).unwrap();
    let mut sensor_right = Vl53l5cx::new_i2c(
        i2c_right, 
        lpn_pin_right,
        tim_right).unwrap();

    sensor_top.off().unwrap();
    sensor_left.off().unwrap();
    sensor_right.off().unwrap();

    sensor_top.init_sensor(address_top).unwrap(); 
    sensor_left.init_sensor(address_left).unwrap(); 
    sensor_right.init_sensor(address_right).unwrap(); 

    sensor_top.set_frequency_hz(30).unwrap();
    sensor_left.set_frequency_hz(30).unwrap();
    sensor_right.set_frequency_hz(30).unwrap();

    sensor_top.start_ranging().unwrap();
    sensor_left.start_ranging().unwrap();
    sensor_right.start_ranging().unwrap();
    
    loop {
        while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
        results_top = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_left.check_data_ready().unwrap() {} // Wait for data to be ready
        results_left = sensor_left.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_right.check_data_ready().unwrap() {} // Wait for data to be ready
        results_right = sensor_right.get_ranging_data().unwrap(); // Get and parse the result data

        write_results_multi(&mut tx, &results_top, &results_left, &results_right, WIDTH); // Print the result to the output
    }

} 
