
use consts::*;
use buffers::*;
use utils::*;

use crate::{buffers, consts, utils, BlockHeader, BusOperation, Error, Vl53l5cx, OutputPin, DelayNs};

impl<B: BusOperation, LPN: OutputPin, RST: OutputPin, T: DelayNs> Vl53l5cx<B, LPN, RST, T> {
    fn poll_for_answer_xtalk(&mut self, address: u16, expected_val: u8) -> Result<(), Error<B::Error>> {
        let mut timeout: u8 = 0;
        while timeout <= 200 {
            if self.temp_buffer[1] == expected_val {
                return Ok(());
            }
            self.read_from_register(address, 4)?;
            self.delay(10); 

            if self.temp_buffer[2] >= 0x7f {
                return Err(Error::Mcu);
            } 
            timeout += 1; 
        }
        Err(Error::Timeout)
    }

    fn program_output_config(&mut self) -> Result<(), Error<B::Error>> {
        let mut header_config: [u32; 2] = [0, 0];
        let resolution = self.get_resolution()?;
        let mut bh: BlockHeader;
        self.data_read_size = 0;
        // Enable mandatory output (meta and common data) 
        let output_bh_enable: [u32; 4] = [
            0x0001FFFF,
            0x00000000,
            0x00000000,
            0xC0000000
            ];
            
            // Send addresses of possible output 
        let mut output: [u32; 17] = [
            0x0000000D,
            0x54000040,
            0x9FD800C0,
            0x9FE40140,
            0x9FF80040,
            0x9FFC0404,
            0xA0FC0100,
            0xA10C0100,
            0xA11C00C0,
            0xA1280902,
            0xA2480040,
            0xA24C0081,
            0xA2540081,
            0xA25C0081,
            0xA2640081,
            0xA26C0084,
            0xA28C0082
        ];
        
        for i in 0..17 {
            if output[i] == 0 || (output_bh_enable[0] & 1 << i) == 0 { 
                continue;
            }
            bh = BlockHeader(output[i]);
            if bh.bh_type() >= 0x1 && bh.bh_type() < 0x0d {
                if bh.bh_idx() >= 0x54d0 && bh.bh_idx() < 0x54d0 + 960 {
                    bh.set_bh_size(resolution as u32);
                } else {
                    bh.set_bh_size(resolution as u32 * VL53L5CX_NB_TARGET_PER_ZONE);
                }
                self.data_read_size += bh.bh_type() * bh.bh_size();
            } else {
                self.data_read_size += bh.bh_size();
            }
            self.data_read_size += 4;
            output[i] = bh.bh_bytes();
        }
        self.data_read_size += 24;

        from_u32_to_u8(&output, &mut self.temp_buffer[..68]);
        self.dci_write_data(VL53L5CX_DCI_OUTPUT_LIST, 68)?;

        header_config[0] = self.data_read_size;
        header_config[1] = 17;

        from_u32_to_u8(&header_config, &mut self.temp_buffer[..8]);
        self.dci_write_data(VL53L5CX_DCI_OUTPUT_CONFIG, 8)?;

        from_u32_to_u8(&output_bh_enable, &mut self.temp_buffer[..16]);
        self.dci_write_data(VL53L5CX_DCI_OUTPUT_ENABLES, 16)?;
       
        Ok(())
    }

    /// This function gets the Xtalk margin. This margin is used to increase the Xtalk threshold. It can also be used to avoid false positives after the Xtalk calibration. The default value is 50 kcps/spads.
    /// 
    /// # Returns
    /// 
    /// * `xtalk_margin` : Xtalk margin in kcps/spads.
    pub fn get_xtalk_margin(&mut self) -> Result<u32, Error<B::Error>> {
        self.dci_read_data(VL53L5CX_DCI_XTALK_CFG, 16)?;
        let mut xtalk_margin: [u32; 1] = [0];
        from_u8_to_u32(&self.temp_buffer[..4], &mut xtalk_margin);
        xtalk_margin[0] /= 2048;

        Ok(xtalk_margin[0])
    }

    /// This function sets the Xtalk margin. This margin is used to increase the Xtalk threshold. It can also be used to avoid false positives after the Xtalk calibration. The default value is 50 kcps/spads.
    /// 
    /// # Arguments
    /// 
    /// * `xtalk_margin` : New Xtalk margin in kcps/spads. Min value is 0 kcps/spads, and max is 10.000 kcps/spads
    pub fn set_xtalk_margin(&mut self, xtalk_margin: u32) -> Result<(), Error<B::Error>> {
        if xtalk_margin > 10000 {
            return Err(Error::InvalidParam);
        }
        let mut margin_kcps: [u8; 4] = [0; 4];
        from_u32_to_u8(&[xtalk_margin*2048], &mut margin_kcps);
        self.dci_replace_data(VL53L5CX_DCI_XTALK_CFG, 16, &margin_kcps, 4, 0)?;

        Ok(())
    }

    /// This function starts the VL53L5CX sensor in order to calibrate Xtalk.
    /// This calibration is recommended is user wants to use a coverglass.
    /// 
    /// # Arguments
    /// 
    /// * `reflectance_percent` : Target reflectance in percent. This value is include between 1 and 99%. For a better efficiency, ST recommends a 3% target reflectance.
    /// * `nb_samples` : Nb of samples used for calibration. A higher number of samples means a higher accuracy, but it increases the calibration time. Minimum is 1 and maximum is 16.
    /// * `distance_mm` : Target distance in mm. The minimum allowed distance is 600mm, and maximum is 3000mm. The target must stay in Full FOV, so short distance are easier for calibration.
    pub fn calibrate_xtalk(&mut self, reflectance_percent: u16, nb_samples: u8, distance_mm: u16) -> Result<(), Error<B::Error>> {
        let mut timeout: u16 = 0;
        let cmd: [u8; 4] = [0x00, 0x03, 0x00, 0x00];
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
        let mut reflectance: [u8; 2] = [0,0];
        let mut distance: [u8;2] = [0,0];
        let samples: [u8;1] = [nb_samples];
        
        // Get initial configuration 
        let resolution = self.get_resolution()?;
        let frequency = self.get_frequency_hz()?;
        let sharpener_percent = self.get_sharpener_percent()?;
        let integration_time_ms = self.get_integration_time()?;
        let target_order = self.get_target_order()?;
        let xtalk_margin = self.get_xtalk_margin()?;
        let ranging_mode = self.get_ranging_mode()?;

        // Check input arguments validity 
        if reflectance_percent < 1 || reflectance_percent > 99
            || distance_mm < 600 || distance_mm > 3000
            || nb_samples < 1 || nb_samples > 16 {
            return Err(Error::InvalidParam);
        }
        self.set_resolution(VL53L5CX_RESOLUTION_8X8)?;

        // Send Xtalk calibration buffer 
        self.temp_buffer[..984].copy_from_slice(&VL53L5CX_CALIBRATE_XTALK);
        self.write_multi_to_register_temp_buffer(0x2c28, 984)?;
        self.poll_for_answer_xtalk(VL53L5CX_UI_CMD_STATUS, 3)?;

        // Format input argument 
        from_u16_to_u8(&[reflectance_percent*16], &mut reflectance);
        from_u16_to_u8(&[distance_mm*4], &mut distance);

        // Update required fields 
        self.dci_replace_data(VL53L5CX_DCI_CAL_CFG, 8, &distance, 2, 0)?;
        self.dci_replace_data(VL53L5CX_DCI_CAL_CFG, 8, &reflectance, 2, 2)?;
        self.dci_replace_data(VL53L5CX_DCI_CAL_CFG, 8, &samples, 1, 4)?;

        // Program output for Xtalk calibration 
        self.program_output_config()?;

        // Start ranging session 
        self.write_multi_to_register(VL53L5CX_UI_CMD_END - (4-1), &cmd)?;
        self.poll_for_answer_xtalk(VL53L5CX_UI_CMD_STATUS, 3)?;

        // Wait for end of calibration 
        while timeout <= 400 {
            self.read_from_register(0, 4)?;
            if self.temp_buffer[0] != VL53L5CX_STATUS_ERROR {
                // Coverglass too good for Xtalk calibration 
                if self.temp_buffer[2] >= 0x7f && self.temp_buffer[3] & 0x80 >> 7 == 1 {
                    self.xtalk_data.copy_from_slice(&VL53L5CX_DEFAULT_XTALK);
                }
                break;
            } else {
                self.delay(50);
                timeout += 1;
            }
        }

        // Save Xtalk data into the Xtalk buffer 
        self.temp_buffer[..72].copy_from_slice(&VL53L5CX_GET_XTALK_CMD);
        self.write_multi_to_register_temp_buffer(0x2fb8, 72)?;
        self.poll_for_answer_xtalk(VL53L5CX_UI_CMD_STATUS, 3)?;
        self.read_from_register(VL53L5CX_UI_CMD_START, VL53L5CX_XTALK_BUFFER_SIZE+4)?;
        self.xtalk_data[..VL53L5CX_XTALK_BUFFER_SIZE-8].copy_from_slice(&self.temp_buffer[8..VL53L5CX_XTALK_BUFFER_SIZE]);
        self.xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE-8..].copy_from_slice(&footer);

        // Reset default buffer 
        self.write_multi_to_register(0x2c34, &VL53L5CX_DEFAULT_CONFIGURATION)?;
        self.poll_for_answer_xtalk(VL53L5CX_UI_CMD_STATUS, 3)?;

        // Reset initial configuration 
        self.set_resolution(resolution)?;
        self.set_frequency_hz(frequency)?;
        self.set_integration_time(integration_time_ms)?;
        self.set_sharpener_percent(sharpener_percent)?;
        self.set_target_order(target_order)?;
        self.set_xtalk_margin(xtalk_margin)?;
        self.set_ranging_mode(ranging_mode)?;

        Ok(())
    }

    /// This function gets the Xtalk buffer. The buffer is available after using the function calibrate_xtalk().
    /// 
    /// # Returns
    /// 
    /// `xtalk_data` : Buffer with a size defined by macro VL53L5CX_XTALK_SIZE.
    pub fn get_caldata_xtalk(&mut self) -> Result<[u8; VL53L5CX_XTALK_BUFFER_SIZE], Error<B::Error>> {
        let footer: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x03, 0x04];
        let mut xtalk_data: [u8; VL53L5CX_XTALK_BUFFER_SIZE] = [0; VL53L5CX_XTALK_BUFFER_SIZE];
        let resolution = self.get_resolution()?;
        self.set_resolution(VL53L5CX_RESOLUTION_8X8)?;

        self.temp_buffer[..72].copy_from_slice(&VL53L5CX_GET_XTALK_CMD);
        self.write_multi_to_register_temp_buffer(0x2fb8, 72)?;
        self.poll_for_answer_xtalk(VL53L5CX_UI_CMD_STATUS, 3)?;
        self.read_from_register(VL53L5CX_UI_CMD_START, VL53L5CX_XTALK_BUFFER_SIZE+4)?;
        xtalk_data[..VL53L5CX_XTALK_BUFFER_SIZE-8].copy_from_slice(&self.temp_buffer[8..VL53L5CX_XTALK_BUFFER_SIZE]);
        xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE-8..].copy_from_slice(&footer);

        self.set_resolution(resolution)?;

        Ok(xtalk_data)
    }

    /// This function sets the Xtalk buffer. 
    /// This function can be used to override default Xtalk buffer.
    /// 
    /// # Arguments
    /// 
    /// `xtalk_data` : Buffer with a size defined by macro VL53L5CX_XTALK_SIZE.
    pub fn set_caldata_xtalk(&mut self, xtalk_data: [u8; VL53L5CX_XTALK_BUFFER_SIZE]) -> Result<(), Error<B::Error>> {
        let resolution = self.get_resolution()?;
        self.xtalk_data.copy_from_slice(&xtalk_data);
        self.set_resolution(resolution)?;

        Ok(())
    }
}