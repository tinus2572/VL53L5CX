use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l5cx, Error, OutputPin, DelayNs};


impl<B: BusOperation, LPN: OutputPin, T: DelayNs> Vl53l5cx<B, LPN, T> {
    
    /// This function gets the current resolution (4x4 or 8x8).
    /// 
    /// # Return
    /// 
    /// `resolution` : Value of this pointer will be equal to 16 for 4x4 mode, and 64 for 8x8 mode.
    pub fn get_resolution(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data(VL53L5CX_DCI_ZONE_CONFIG, 8)?;
        let resolution: u8 = self.temp_buffer[0x00] * self.temp_buffer[0x01];

        Ok(resolution)
    }

    /// This function sets a new resolution (4x4 or 8x8).
    /// 
    /// # Arguments
    /// 
    /// * `resolution` : Use macro VL53L5CX_RESOLUTION_4X4 or VL53L5CX_RESOLUTION_8X8 to set the resolution.
    pub fn set_resolution(&mut self, resolution: u8) -> Result<(), Error<B::Error>> {

        if resolution == VL53L5CX_RESOLUTION_4X4 {
            self.dci_read_data(VL53L5CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 64;
            self.temp_buffer[0x06] = 64;
            self.temp_buffer[0x09] = 4;
            self.dci_write_data(VL53L5CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data(VL53L5CX_DCI_ZONE_CONFIG, 8)?;
            self.temp_buffer[0x00] = 4;
            self.temp_buffer[0x01] = 4;
            self.temp_buffer[0x04] = 8;
            self.temp_buffer[0x05] = 8;
            self.dci_write_data(VL53L5CX_DCI_ZONE_CONFIG, 8)?;
        } else if resolution == VL53L5CX_RESOLUTION_8X8 {
            self.dci_read_data(VL53L5CX_DCI_DSS_CONFIG, 16)?;
            self.temp_buffer[0x04] = 16;
            self.temp_buffer[0x06] = 16;
            self.temp_buffer[0x09] = 1;
            self.dci_write_data(VL53L5CX_DCI_DSS_CONFIG, 16)?;
            self.dci_read_data(VL53L5CX_DCI_ZONE_CONFIG, 8)?;
            self.temp_buffer[0x00] = 8;
            self.temp_buffer[0x01] = 8;
            self.temp_buffer[0x04] = 4;
            self.temp_buffer[0x05] = 4;
            self.dci_write_data(VL53L5CX_DCI_ZONE_CONFIG, 8)?;
        } else {
            return Err(Error::InvalidParam);
        }
        self.send_offset_data(resolution)?;
        self.send_xtalk_data(resolution)?;

        Ok(())
    }

    /// This function is used to get the current sensor power mode.
    /// 
    /// # Return
    /// 
    /// `power_mode` : Current power mode. The value of this is equal to 0 if the sensor is in low power, (VL53L5CX_POWER_MODE_SLEEP), or 1 if sensor is in standard mode (VL53L5CX_POWER_MODE_WAKEUP).
    pub fn get_power_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let power_mode: u8;
        self.write_to_register(0x7fff, 0x00)?;
        self.read_from_register(0x009, 1)?;    
        if self.temp_buffer[0] == 0x4 {
            power_mode = VL53L5CX_POWER_MODE_WAKEUP;
        } else if self.temp_buffer[0] == 0x2 {
            power_mode = VL53L5CX_POWER_MODE_SLEEP;
        } else {
            return Err(Error::Other);
        }
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(power_mode)
    }

    /// This function is used to set the sensor in Low Power mode, for example if the sensor is not used during a long time. The macro VL53L5CX_POWER_MODE_SLEEP can be used to enable the low power mode. When user want to restart the sensor, he can use macro VL53L5CX_POWER_MODE_WAKEUP. Please ensure that the device is not streaming before calling the function.
    /// 
    /// # Arguments
    /// 
    /// * `power_mode` : Selected power mode (VL53L5CX_POWER_MODE_SLEEP or VL53L5CX_POWER_MODE_WAKEUP)
    pub fn set_power_mode(&mut self, power_mode: u8) -> Result<(), Error<B::Error>> {
        let current_power_mode: u8 = self.get_power_mode()?;
        if power_mode != current_power_mode {
            if power_mode == VL53L5CX_POWER_MODE_WAKEUP {
                self.write_to_register(0x7fff, 0x00)?;
                self.write_to_register(0x09, 0x04)?;
                self.poll_for_answer(1, 0, 0x06, 0x01, 1)?;
            } else if power_mode == VL53L5CX_POWER_MODE_SLEEP {
                self.write_to_register(0x7fff, 0x00)?;
                self.write_to_register(0x09, 0x02)?;
                self.poll_for_answer(1, 0, 0x06, 0x01, 0)?;
            } else {
                return Err(Error::Other);
            }
        }
        self.write_to_register(0x7fff, 0x02)?;
        
        Ok(())
    }
    
    /// This function gets the current target order (closest or strongest).
    ///
    /// # Return
    /// 
    /// `target_order` : Contains the target order.
    pub fn get_target_order(&mut self) -> Result<u8, Error<B::Error>> {
        let target_order: u8;
        self.dci_read_data(VL53L5CX_DCI_TARGET_ORDER, 4)?;
        target_order = self.temp_buffer[0];

        Ok(target_order)
    }

    /// This function sets a new target order. Please use macros VL53L5CX_TARGET_ORDER_STRONGEST and VL53L5CX_TARGET_ORDER_CLOSEST to define the new output order. By default, the sensor is configured with the strongest output.
    /// 
    /// # Arguments
    /// 
    /// * `target_order` : Required target order.\
    pub fn set_target_order(&mut self, target_order: u8) -> Result<(), Error<B::Error>> {
        if target_order == VL53L5CX_TARGET_ORDER_CLOSEST || target_order == VL53L5CX_TARGET_ORDER_STRONGEST {
            self.dci_replace_data(VL53L5CX_DCI_TARGET_ORDER, 4, &[target_order], 1, 0x0)?;
        } else {
            return Err(Error::InvalidParam);
        }
        Ok(())
    }

    /// This function gets the current sharpener in percent. Sharpener can be changed to blur more or less zones depending of the application.
    ///
    /// # Return
    /// 
    /// `sharpener_percent` : Contains the sharpener in percent.
    pub fn get_sharpener_percent(&mut self) -> Result<u32, Error<B::Error>> {
        let sharpener_percent: u32;
        self.dci_read_data(VL53L5CX_DCI_SHARPENER, 16)?;
        sharpener_percent = self.temp_buffer[0xD] as u32* 100 / 255;

        Ok(sharpener_percent)
    }
    
    /// This function sets a new sharpener value in percent. Sharpener can be changed to blur more or less zones depending of the application. Min value is 0 (disabled), and max is 99.
    /// 
    /// # Arguments
    /// 
    /// * `sharpener_percent` : Value between 0 (disabled) and 99%.
    pub fn set_sharpener_percent(&mut self, sharpener_percent: u32) -> Result<(), Error<B::Error>> {
        let sharpener: u32;
        if sharpener_percent >= 100 {
            return Err(Error::InvalidParam);
        }
        sharpener = sharpener_percent * 255 / 100;
        self.dci_replace_data(VL53L5CX_DCI_SHARPENER, 16, &[sharpener as u8], 1, 0xd)?;

        Ok(())
    }

    /// This function gets the current integration time in ms.
    /// 
    /// # Return
    /// 
    /// `time_ms`: Contains integration time in ms.
    pub fn get_integration_time(&mut self) -> Result<u32, Error<B::Error>> {
        let mut time_ms: [u32; 1] = [0];
        self.dci_read_data(VL53L5CX_DCI_INT_TIME, 20)?;
        from_u8_to_u32(&self.temp_buffer[..4], &mut time_ms);
        
        time_ms[0] /= 1000;
        
        Ok(time_ms[0])
    }

    /// This function sets a new integration time in ms. Integration time must be computed to be lower than the ranging period, for a selected resolution. Please note that this function has no impact on ranging mode continuous.
    /// 
    /// # Arguments
    /// 
    /// * `time_ms` : Contains the integration time in ms. For all resolutions and frequency, the minimum value is 2ms, and the maximum is 1000ms.
    pub fn set_integration_time(&mut self, integration_time_ms: u32) -> Result<(), Error<B::Error>> {
        let mut integration: u32 = integration_time_ms;

        // Integration time must be between 2ms and 1000ms 
        if integration < 2 || integration > 1000 {
            return Err(Error::InvalidParam);
        }
        integration *= 1000;
        
        let mut buf: [u8; 4] = [0; 4];
        let tmp: [u32; 1] = [integration];
        from_u32_to_u8(&tmp, &mut buf);
        
        self.dci_replace_data(VL53L5CX_DCI_INT_TIME, 20, &buf, 4, 0x00)?;

        Ok(())
    }

    /// This function is used to get the ranging mode. Two modes are available using ULD : Continuous and autonomous. The default mode is Autonomous.
    ///
    /// # Return
    /// 
    /// `ranging_mode` : current ranging mode
    pub fn get_ranging_mode(&mut self) -> Result<u8, Error<B::Error>> {
        let ranging_mode: u8;
        self.dci_read_data(VL53L5CX_DCI_RANGING_MODE, 8)?;
        if self.temp_buffer[1] == 1 {
            ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;
        } else {
            ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS;
        }
        Ok(ranging_mode)
    }

    /// This function is used to set the ranging mode. Two modes are available using ULD : Continuous and autonomous. The default mode is Autonomous.
    ///
    /// # Arguments
    /// 
    /// * `ranging_mode` : Use macros VL53L5CX_RANGING_MODE_CONTINUOUS, VL53L5CX_RANGING_MODE_AUTONOMOUS.
    pub fn set_ranging_mode(&mut self, ranging_mode: u8) -> Result<(), Error<B::Error>> {
        let mut single_range: [u32; 1] = [0];
        self.dci_read_data(VL53L5CX_DCI_RANGING_MODE, 8)?;

        if ranging_mode == VL53L5CX_RANGING_MODE_CONTINUOUS {
            self.temp_buffer[1] = 1;
            self.temp_buffer[3] = 3;
            single_range[0] = 0;
        } else if ranging_mode == VL53L5CX_RANGING_MODE_AUTONOMOUS {
            self.temp_buffer[1] = 3;
            self.temp_buffer[3] = 2;
            single_range[0] = 1;
        } else {
            return Err(Error::Other);
        }

        self.dci_write_data(VL53L5CX_DCI_RANGING_MODE, 8)?;

        from_u32_to_u8(&single_range, &mut self.temp_buffer[..4]);
        self.dci_write_data(VL53L5CX_DCI_SINGLE_RANGE, 4)?;
        
        Ok(())
    }

    /// This function is used to enable the VCSEL charge pump
    pub fn enable_internal_cp(&mut self) -> Result<(), Error<B::Error>> {
        let vcsel_bootup_fsm: [u8;1] = [1];
    	let analog_dynamic_pad_0: [u8;1] = [0];

	    self.dci_replace_data(VL53L5CX_DCI_INTERNAL_CP, 16, &vcsel_bootup_fsm, 1, 0x0A)?;

	    self.dci_replace_data(VL53L5CX_DCI_INTERNAL_CP, 16, &analog_dynamic_pad_0, 1, 0x0e)?;

	    Ok(())
    }
    
    /// This function is used to disable the VCSEL charge pump
    /// This optimizes the power consumption of the device
    /// To be used only if AVDD = 3.3V
    pub fn disable_internal_cp(&mut self) -> Result<(), Error<B::Error>> {
        let vcsel_bootup_fsm: [u8;1] = [0];
    	let analog_dynamic_pad_0: [u8;1] = [1];

	    self.dci_replace_data(VL53L5CX_DCI_INTERNAL_CP, 16, &vcsel_bootup_fsm, 1, 0x0A)?;

	    self.dci_replace_data(VL53L5CX_DCI_INTERNAL_CP, 16, &analog_dynamic_pad_0, 1, 0x0e)?;

	    Ok(())
    }



    /// This function is used to get the number of frames between 2 temperature compensation.
    /// 
    /// # Arguments
    /// 
    /// * `repeat_count` : Number of frames before next temperature compensation. Set to 0 to disable the feature (default configuration).
    pub fn get_vhv_repeat_count(&mut self) -> Result<u32, Error<B::Error>> {
        self.dci_read_data(VL53L5CX_DCI_VHV_CONFIG, 16)?;
        let mut repeat_count: [u32; 1] = [0];
        from_u8_to_u32(&self.temp_buffer[4..8], &mut repeat_count);
        Ok(repeat_count[0])
    }
    
    /// This function is used to set a periodic temperature compensation. By setting a repeat count different to 0 the firmware automatically runs a temperature calibration every N frames. Default the repeat count is set to 0
    /// 
    /// # Arguments
    /// 
    /// * `repeat_count` : Number of frames between temperature compensation. Set to 0 to disable the feature (default configuration).
        pub fn set_vhv_repeat_count(&mut self, repeat_count: u32) -> Result<(), Error<B::Error>> {
        let mut tmp: [u8;4] = [0;4];
        from_u32_to_u8(&[repeat_count], &mut tmp);
        self.dci_replace_data(VL53L5CX_DCI_VHV_CONFIG, 16, &tmp, 4, 0x4)?;
        Ok(())
    }


    /// This function gets the current ranging frequency in Hz. Ranging frequency corresponds to the time between each measurement.
    ///
    /// # Return
    /// 
    /// `frequency_hz` : Contains the ranging frequency in Hz.
    pub fn get_frequency_hz(&mut self) -> Result<u8, Error<B::Error>> {
        self.dci_read_data(VL53L5CX_DCI_FREQ_HZ, 4)?;
        let frequency_hz: u8 = self.temp_buffer[0x01];

        Ok(frequency_hz)
    }

    /// This function sets a new ranging frequency in Hz. Ranging frequency corresponds to the measurements frequency. This setting depends of the resolution, so please select your resolution before using this function.
    /// 
    /// # Arguments
    /// 
    /// * `frequency_hz` : Contains the ranging frequency in Hz.
    ///  - For 4x4, min and max allowed values are : 1 to 60
    ///  - For 8x8, min and max allowed values are : 1 to 15
    pub fn set_frequency_hz(&mut self, frequency_hz: u8) -> Result<(), Error<B::Error>> {
        let tmp: [u8; 1] = [frequency_hz];
        self.dci_replace_data(VL53L5CX_DCI_FREQ_HZ, 4, &tmp, 1, 0x01)?;

        Ok(())
    }
}