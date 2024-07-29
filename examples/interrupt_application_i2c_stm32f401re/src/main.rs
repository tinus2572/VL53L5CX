#![no_std]
#![no_main]

use vl53l5cx::{
    consts::VL53L5CX_DEFAULT_I2C_ADDRESS, ResultsData, Vl53l5cx
};

use panic_halt as _; 
use cortex_m_rt::entry;
use cortex_m::{interrupt::Mutex, asm::wfi};


use core::{fmt::Write, cell::RefCell};

use embedded_hal::i2c::SevenBitAddress;

use stm32f4xx_hal::{
    gpio::{
        self,
        Input,
        Output, 
        Pin, 
        PinState::{High, Low},
        Edge,
        gpioa, 
        gpiob,
        Alternate}, 
    pac::{USART2, Peripherals, CorePeripherals, interrupt, TIM1}, 
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

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData, width: usize) {

    writeln!(tx, "\x1B[2H").unwrap();

    writeln!(tx, "VL53L5A1 Interrupt Ranging demo application\n").unwrap();
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

static INT_PIN: Mutex<RefCell<Option<gpio::PA4<Input>>>> = Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    let mut results: ResultsData = ResultsData::new();
    
    let mut dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    let _delay: SysDelay = cp.SYST.delay(&clocks);
    let tim_top: Delay<TIM1, 1000> = dp.TIM1.delay_ms(&clocks);


    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    
    let _pwr_pin: Pin<'B', 0, Output> = gpiob.pb0.into_push_pull_output_in_state(High);
    let i2c_rst_pin: Pin<'B', 3, Output> = gpiob.pb3.into_push_pull_output_in_state(Low);
    let lpn_pin: Pin<'B', 4, Output> = gpiob.pb4.into_push_pull_output_in_state(High);
    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
     
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(460800.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap();
    
    let mut int_pin: Pin<'A', 4> = gpioa.pa4.into_input().internal_pull_up(true);
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source  
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    let resolution: u8 = (WIDTH * WIDTH) as u8;

    
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    
    let i2c: StmI2c<I2C1> = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard{frequency:200.kHz()},
        &clocks);
        
    let i2c_bus: RefCell<StmI2c<I2C1>> = RefCell::new(i2c);
    let address: SevenBitAddress = VL53L5CX_DEFAULT_I2C_ADDRESS;
        
    let mut sensor_top = Vl53l5cx::new_i2c(
        RefCellDevice::new(&i2c_bus), 
            lpn_pin,
            i2c_rst_pin,
            tim_top
        ).unwrap();

    sensor_top.init_sensor(address).unwrap(); 
    
    sensor_top.set_resolution(resolution).unwrap();
    sensor_top.start_ranging().unwrap();

    write_results(&mut tx, &results, WIDTH);
    
    loop {
        wfi(); // Wait for data to be ready
        results = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data
        write_results(&mut tx, &results, WIDTH); // Print the result to the output
    }

} 

#[interrupt]
fn EXTI4() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain Access to Global Data
        //INTERRUPT.borrow(cs).set(true);
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}