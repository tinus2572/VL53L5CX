use consts::*;
use crate::{consts, Vl53l5cx, Error, SevenBitAddress, I2c, OutputPin, DelayNs};

pub trait BusOperation {
    type Error;
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>; 
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct Vl53l5cxI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Vl53l5cxI2C<P> {
    pub(crate) fn new(i2c: P) -> Self {
        Vl53l5cxI2C { i2c: i2c, address: VL53L5CX_DEFAULT_I2C_ADDRESS }
    }
}

impl<P: I2c> BusOperation for Vl53l5cxI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;
        
        Ok(())
    }
    
    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }
    
    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;
        
        Ok(())
    }
}

impl<P, LPN, RST, T> Vl53l5cx<Vl53l5cxI2C<P>, LPN, RST, T>
    where
    P: I2c,
    LPN: OutputPin,
    RST: OutputPin,
    T: DelayNs
{
    pub fn new_i2c(i2c: P, lpn_pin: LPN, i2c_rst_pin: RST, tim: T) -> Result<Self, Error<P::Error>> 
    {
        Ok(Vl53l5cx { 
            temp_buffer: [0; VL53L5CX_TEMPORARY_BUFFER_SIZE],
            offset_data: [0; VL53L5CX_OFFSET_BUFFER_SIZE],
            xtalk_data: [0; VL53L5CX_XTALK_BUFFER_SIZE],
            streamcount: 0,
            data_read_size: 0,
            is_auto_stop_enabled: false,
            lpn_pin: lpn_pin,
            i2c_rst_pin: i2c_rst_pin,
            bus: Vl53l5cxI2C::new(i2c),
            tim: tim,
            chunk_size: I2C_CHUNK_SIZE
        })
    }
    
    pub fn set_i2c_address(&mut self, i2c_address: SevenBitAddress) -> Result<(), Error<P::Error>> {
        self.write_to_register(0x7fff, 0x00)?;
        self.write_to_register(0x4, i2c_address)?;
        self.bus.address = i2c_address;
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }
    
    pub fn i2c_reset(&mut self) -> Result<(), Error<P::Error>> {
        self.i2c_rst_pin.set_low().unwrap();
        
        Ok(())
    }



    pub fn init_sensor(&mut self, address: u8) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        if address != self.bus.address {
            self.set_i2c_address(address)?;
        }
        self.is_alive()?;
        self.init()?;
        Ok(())
    }
}


