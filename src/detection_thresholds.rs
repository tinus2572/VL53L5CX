
use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l5cx, Error, OutputPin, DelayNs};

/// Structure DetectionThresholds contains a single threshold. This structure  is never used alone, it must be used as an array of 64 thresholds (defined by macro VL53L5CX_NB_THRESHOLDS).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DetectionThresholds {
    pub param_low_thresh: i32,
    pub param_high_thresh: i32,
    pub measurement: u8,
    pub th_type: u8,
    pub zone_num: u8,
    pub math_op: u8
}

impl DetectionThresholds {
    pub fn new() -> Self {
        DetectionThresholds { param_low_thresh: 0,
            param_high_thresh: 0,
            measurement: VL53L5CX_DISTANCE_MM,
            th_type: VL53L5CX_IN_WINDOW,
            zone_num: 0,
            math_op: VL53L5CX_OPERATION_NONE }
    }
}

fn from_u8_to_thresholds(src: &[u8], dst: &mut [DetectionThresholds]) {
    for i in 0..dst.len() {
        let j: usize = 12 * i;
        let mut tmp : [i32;1] = [0];
        from_u8_to_i32(&src[j..j+4], &mut tmp);
        dst[i].param_low_thresh = tmp[0];
        from_u8_to_i32(&src[j+4..j+8], &mut tmp);
        dst[i].param_high_thresh = tmp[0];
        dst[i].measurement = src[j+8];
        dst[i].th_type = src[j+9];
        dst[i].zone_num = src[j+10];
        dst[i].math_op = src[j+11];
    }
}

fn from_thresholds_to_u8(src: &[DetectionThresholds], dst: &mut [u8]) {
    for i in 0..src.len() {
        let j: usize = 12 * i;
        from_i32_to_u8(&[src[i].param_low_thresh], &mut dst[j..j+4]);
        from_i32_to_u8(&[src[i].param_high_thresh], &mut dst[j+4..j+8]);
        dst[j+8] = src[i].measurement;
        dst[j+9] = src[i].th_type;
        dst[j+10] = src[i].zone_num;
        dst[j+11] = src[i].math_op;
    }
}

impl<B: BusOperation, LPN: OutputPin, RST: OutputPin, T: DelayNs> Vl53l5cx<B, LPN, RST, T> {

    /// This function allows indicating if the detection thresholds are enabled.
    /// 
    /// # Return
    /// 
    /// * `enabled` : Set to 1 if enabled, or 0 if disable.
    pub fn get_detection_thresholds_enable(&mut self) -> Result<u8, Error<B::Error>> {
        let enabled: u8;
        self.dci_read_data(VL53L5CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8)?;
        enabled = self.temp_buffer[0x1];
        Ok(enabled)
    }
    
    /// This function allows enable the detection thresholds.
    /// 
    /// # Arguments
    /// 
    /// * `enabled` : Set to 1 to enable, or 0 to disable thresholds.
    pub fn set_detection_thresholds_enable(&mut self, enabled: u8) -> Result<(), Error<B::Error>> {
        let mut grp_global_config: [u8; 4] = [0x01, 0x00, 0x01, 0x00];
        let mut tmp: [u8; 1] = [0];
        if enabled == 1 {
            grp_global_config[0x01] = 0x01;
            tmp[0] = 0x04;
        } else {
            grp_global_config[0x01] = 0x00;
            tmp[0] = 0x0C;
        }
        
        // Set global interrupt config 
        self.dci_replace_data(VL53L5CX_DCI_DET_THRESH_GLOBAL_CONFIG, 8, &grp_global_config, 4, 0x00)?;
        
        // Update interrupt config 
        self.dci_replace_data(VL53L5CX_DCI_DET_THRESH_CONFIG, 20, &tmp, 1, 0x11)?;
        
        Ok(())
    }

    /// This function allows getting the detection thresholds.
    /// 
    /// # Return
    /// 
    /// * `thresholds` : Array of 64 thresholds.
    pub fn get_detection_thresholds(&mut self) -> Result<[DetectionThresholds; VL53L5CX_NB_THRESHOLDS], Error<B::Error>> {
        let mut thresholds: [DetectionThresholds; VL53L5CX_NB_THRESHOLDS] = [DetectionThresholds::new(); VL53L5CX_NB_THRESHOLDS];
        
        // Get thresholds configuration 
        self.dci_read_data(VL53L5CX_DCI_DET_THRESH_START, VL53L5CX_NB_THRESHOLDS * 12)?;
        from_u8_to_thresholds(&self.temp_buffer[..VL53L5CX_NB_THRESHOLDS * 12], &mut thresholds);
        
        for i in 0..VL53L5CX_NB_THRESHOLDS {
            if thresholds[i].measurement == VL53L5CX_DISTANCE_MM {
                thresholds[i].param_low_thresh  /= 4;
                thresholds[i].param_high_thresh /= 4;
            } else if thresholds[i].measurement == VL53L5CX_SIGNAL_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  /= 2048;
                thresholds[i].param_high_thresh /= 2048;
            } else if thresholds[i].measurement == VL53L5CX_RANGE_SIGMA_MM {
                thresholds[i].param_low_thresh  /= 128;
                thresholds[i].param_high_thresh /= 128;
            } else if thresholds[i].measurement == VL53L5CX_AMBIENT_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  /= 2048;
                thresholds[i].param_high_thresh /= 2048;
            } else if thresholds[i].measurement == VL53L5CX_NB_SPADS_ENABLED {
                thresholds[i].param_low_thresh  /= 256;
                thresholds[i].param_high_thresh /= 256;
            } else if thresholds[i].measurement == VL53L5CX_MOTION_INDICATOR {
                thresholds[i].param_low_thresh  /= 65535;
                thresholds[i].param_high_thresh /= 65535;
            }
        }
        Ok(thresholds)
    }

    /// This function allows programming the detection thresholds.
    /// 
    /// # Arguments
    /// 
    /// * `thresholds` :  Array of 64 thresholds.
    pub fn set_detection_thresholds(&mut self, thresholds: &mut [DetectionThresholds; VL53L5CX_NB_THRESHOLDS] ) -> Result<(), Error<B::Error>> {
        for i in 0..VL53L5CX_NB_THRESHOLDS {
            if thresholds[i].measurement == VL53L5CX_DISTANCE_MM {
                thresholds[i].param_low_thresh  *= 4;
                thresholds[i].param_high_thresh *= 4;
            } else if thresholds[i].measurement == VL53L5CX_SIGNAL_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  *= 2048;
                thresholds[i].param_high_thresh *= 2048;
            } else if thresholds[i].measurement == VL53L5CX_RANGE_SIGMA_MM {
                thresholds[i].param_low_thresh  *= 128;
                thresholds[i].param_high_thresh *= 128;
            } else if thresholds[i].measurement == VL53L5CX_AMBIENT_PER_SPAD_KCPS {
                thresholds[i].param_low_thresh  *= 2048;
                thresholds[i].param_high_thresh *= 2048;
            } else if thresholds[i].measurement == VL53L5CX_NB_SPADS_ENABLED {
                thresholds[i].param_low_thresh  *= 256;
                thresholds[i].param_high_thresh *= 256;
            } else if thresholds[i].measurement == VL53L5CX_MOTION_INDICATOR {
                thresholds[i].param_low_thresh  *= 65535;
                thresholds[i].param_high_thresh *= 65535;
            }
        } 

        // Set valid target list 
        self.temp_buffer[..8].copy_from_slice(&[0x05; 8]);
        self.dci_write_data(VL53L5CX_DCI_DET_THRESH_VALID_STATUS, 8)?;
        
        // Set thresholds configuration 
        from_thresholds_to_u8(thresholds, &mut self.temp_buffer[..VL53L5CX_NB_THRESHOLDS * 12]);
        self.dci_write_data(VL53L5CX_DCI_DET_THRESH_START, VL53L5CX_NB_THRESHOLDS * 12)?;

        Ok(())
    }

    /// This function is used to enable or disable the auto-stop feature. When ToF runs in autonomous mode with detection threshold, the sensor only emits an interrupt (INT pin) when a threshold is reached. Interrupt is raised when the measurement is completed. It is possible to abort the ranging without waiting for end of measurement completed by enabling the auto-stop. The sensor emits an interrupt and quickly aborts the measurements in progress. Please note that stop_ranging() function needs to be used after interrupt raised for a clean stop. This function is used to get the auto_stop flag.
    /// 
    /// # Returns
    /// 
    /// * `auto_stop` :  auto-stop feature, false if disabled (default)
    pub fn get_detection_thresholds_auto_stop(&mut self) -> Result<bool, Error<B::Error>> {
        self.dci_read_data(VL53L5CX_DCI_PIPE_CONTROL, 4)?;
        let auto_stop: bool = self.temp_buffer[0x03] != 0;
        Ok(auto_stop)
    }

    /// This function is used to enable or disable the auto-stop feature. When ToF runs in autonomous mode with detection threshold, the sensor only emits an interrupt (INT pin) when a threshold is reached. Interrupt is raised when the measurement is completed. It is possible to abort the ranging without waiting for end of measurement completed by enabling the auto-stop. The sensor emits an interrupt and quickly aborts the measurements in progress. Please note that stop_ranging() function needs to be used after interrupt raised for a clean stop. This function is used to set the auto_stop flag.
    /// 
    /// # Arguments
    /// 
    /// * `auto_stop` :  auto-stop feature, false if disabled (default)
    pub fn set_detection_thresholds_auto_stop(&mut self, auto_stop: bool) -> Result<(), Error<B::Error>> {
        let tmp: [u8; 1] = if auto_stop {[1]} else {[0]};
        self.dci_replace_data(VL53L5CX_DCI_PIPE_CONTROL, 4, &tmp, 1, 0x03)?;
        self.is_auto_stop_enabled = auto_stop;
        Ok(())
    }
}