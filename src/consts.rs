use embedded_hal::i2c::SevenBitAddress;

// Default I2C address of VL53L5CX sensor. 
// Can be changed using function set_i2c_address() function is called.
pub const VL53L5CX_DEFAULT_I2C_ADDRESS: SevenBitAddress = 0x52 >> 1;

// Macro VL53L5CX_RESOLUTION_4X4 or VL53L5CX_RESOLUTION_8X8 allows
// setting sensor in 4x4 mode or 8x8 mode, 
// using function set_resolution().
pub const VL53L5CX_RESOLUTION_4X4: u8 = 16;
pub const VL53L5CX_RESOLUTION_8X8: u8 = 64;

// Macro VL53L5CX_STATUS_OK indicates that VL53L5 sensor has no error.
// VL53L5CX_STATUS_ERROR indicates that something is wrong (value, I2C access, ...). 
// Macro VL53L5CX_MCU_ERROR is used to indicate a MCU issue.
pub(crate) const VL53L5CX_STATUS_OK: u8 = 0;
pub(crate) const VL53L5CX_STATUS_TIMEOUT_ERROR: u8 = 1;
pub(crate) const VL53L5CX_STATUS_CORRUPTED_FRAME: u8 = 2;
pub(crate) const VL53L5CX_STATUS_LASER_SAFETY: u8 = 3;
pub(crate) const VL53L5CX_MCU_ERROR: u8 = 66;
pub(crate) const VL53L5CX_STATUS_INVALID_PARAM: u8 = 127;
pub(crate) const VL53L5CX_STATUS_ERROR: u8 = 255;

// Definitions for Range results block headers
pub(crate) const VL53L5CX_START_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x0000000D } else { 0x0000000D };
pub(crate) const VL53L5CX_METADATA_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x54B400C0 } else { 0x54B400C0 };
pub(crate) const VL53L5CX_COMMONDATA_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x54C00040 } else { 0x54C00040 };
pub(crate) const VL53L5CX_AMBIENT_RATE_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x54D00104 } else { 0x54D00104 };
pub(crate) const VL53L5CX_SPAD_COUNT_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x55D00404 } else { 0x57D00401 };
pub(crate) const VL53L5CX_NB_TARGET_DETECTED_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDB840401 } else { 0x55D00404 };
pub(crate) const VL53L5CX_SIGNAL_RATE_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDBC40404 } else { 0x58900404 };
pub(crate) const VL53L5CX_RANGE_SIGMA_MM_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDEC40402 } else { 0x64900402 };
pub(crate) const VL53L5CX_DISTANCE_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDF440402 } else { 0x66900402 };
pub(crate) const VL53L5CX_REFLECTANCE_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xE0440401 } else { 0x6A900401 };
pub(crate) const VL53L5CX_TARGET_STATUS_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xE0840401 } else { 0x6B900401 };
pub(crate) const VL53L5CX_MOTION_DETECT_BH: u32 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xD85808C0 } else { 0xCC5008C0 };

pub(crate) const VL53L5CX_METADATA_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x54B4 } else { 0x54B4 };
pub(crate) const VL53L5CX_SPAD_COUNT_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x55D0 } else { 0x55D0 };
pub(crate) const VL53L5CX_AMBIENT_RATE_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0x54D0 } else { 0x54D0 };
pub(crate) const VL53L5CX_NB_TARGET_DETECTED_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDB84 } else { 0x57D0 };
pub(crate) const VL53L5CX_SIGNAL_RATE_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDBC4 } else { 0x5890 };
pub(crate) const VL53L5CX_RANGE_SIGMA_MM_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDEC4 } else { 0x6490 };
pub(crate) const VL53L5CX_DISTANCE_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xDF44 } else { 0x6690 };
pub(crate) const VL53L5CX_REFLECTANCE_EST_PC_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xE044 } else { 0x6A90 };
pub(crate) const VL53L5CX_TARGET_STATUS_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xE084 } else { 0x6B90 };
pub(crate) const VL53L5CX_MOTION_DETEC_IDX: u16 = if VL53L5CX_NB_TARGET_PER_ZONE == 1 { 0xD858 } else { 0xCC50 };

pub(crate) const VL53L5CX_NVM_DATA_SIZE: usize = 492;
pub(crate) const VL53L5CX_CONFIGURATION_SIZE: usize = 972;
pub(crate) const VL53L5CX_OFFSET_BUFFER_SIZE: usize = 488;
pub(crate) const VL53L5CX_XTALK_BUFFER_SIZE: usize = 776;

// Macro VL53L5CX_TARGET_ORDER_STRONGEST or VL53L5CX_TARGET_ORDER_CLOSEST
// are used to select the target order for data output.
pub const VL53L5CX_TARGET_ORDER_CLOSEST: u8 = 1;
pub const VL53L5CX_TARGET_ORDER_STRONGEST: u8 = 2;

// Macro VL53L5CX_RANGING_MODE_CONTINUOUS and
// VL53L5CX_RANGING_MODE_AUTONOMOUS are used to change the ranging mode.
// Autonomous mode can be used to set a precise integration time, whereas
// continuous is always maximum.
pub const VL53L5CX_RANGING_MODE_CONTINUOUS: u8 = 1;
pub const VL53L5CX_RANGING_MODE_AUTONOMOUS: u8 = 3;

// The default power mode is VL53L5CX_POWER_MODE_WAKEUP. User can choose two different modes to save power consumption when is the device is not used:
// - VL53L5CX_POWER_MODE_SLEEP: This mode retains the firmware and the configuration. It is recommended when the device needs to quickly wake-up.
// Both modes can be changed using function set_power_mode().
pub const VL53L5CX_POWER_MODE_SLEEP: u8 = 0;
pub const VL53L5CX_POWER_MODE_WAKEUP: u8 = 1;

// Inner Macro for API. Not for user, only for development.
pub(crate) const VL53L5CX_DCI_ZONE_CONFIG: u16 = 0x5450;
pub(crate) const VL53L5CX_DCI_FREQ_HZ: u16 = 0x5458;
pub(crate) const VL53L5CX_DCI_INT_TIME: u16 = 0x545C;
pub(crate) const VL53L5CX_DCI_FW_NB_TARGET: u16 = 0x547;
pub(crate) const VL53L5CX_DCI_RANGING_MODE: u16 = 0xAD30;
pub(crate) const VL53L5CX_DCI_DSS_CONFIG: u16 = 0xAD38;
pub(crate) const VL53L5CX_DCI_VHV_CONFIG: u16 = 0xAD60;
pub(crate) const VL53L5CX_DCI_TARGET_ORDER: u16 = 0xAE64;
pub(crate) const VL53L5CX_DCI_SHARPENER: u16 = 0xAED8;
pub(crate) const VL53L5CX_DCI_INTERNAL_CP: u16 = 0xB39C;
pub(crate) const VL53L5CX_DCI_SYNC_PIN: u16 = 0xB5F0;
pub(crate) const VL53L5CX_DCI_MOTION_DETECTOR_CFG: u16 = 0xBFAC;
pub(crate) const VL53L5CX_DCI_SINGLE_RANGE: u16 = 0xD964;
pub(crate) const VL53L5CX_DCI_OUTPUT_CONFIG: u16 = 0xD968;
pub(crate) const VL53L5CX_DCI_OUTPUT_ENABLES: u16 = 0xD970;
pub(crate) const VL53L5CX_DCI_OUTPUT_LIST: u16 = 0xD980;
pub(crate) const VL53L5CX_DCI_PIPE_CONTROL: u16 = 0xDB80;

pub(crate) const VL53L5CX_GLARE_FILTER: u16 = 0xE108;

pub(crate) const VL53L5CX_UI_CMD_STATUS: u16 = 0x2C00;
pub(crate) const VL53L5CX_UI_CMD_START: u16 = 0x2C04;
pub(crate) const VL53L5CX_UI_CMD_END: u16 = 0x2FFF;

// Inner values for API. Max buffer size depends of the selected output.
const L5CX_AMB_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_AMBIENT_PER_SPAD") { 0 } else { 260 };
const L5CX_SPAD_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_NB_SPADS_ENABLED") { 0 } else { 260 };
const L5CX_NTAR_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_NB_TARGET_DETECTED") { 0 } else { 68 };
const L5CX_SPS_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_SIGNAL_PER_SPAD") { 0 } else { 256 * VL53L5CX_NB_TARGET_PER_ZONE as usize + 4 };
const L5CX_SIGR_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_RANGE_SIGMA_MM") { 0 } else { 256 * VL53L5CX_NB_TARGET_PER_ZONE as usize + 4 };
const L5CX_DIST_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_DISTANCE_MM") { 0 } else { 256 * VL53L5CX_NB_TARGET_PER_ZONE as usize + 4 };
const L5CX_RFLEST_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_REFLECTANCE_PERCENT") { 0 } else { 256 * VL53L5CX_NB_TARGET_PER_ZONE as usize + 4 };
const L5CX_STA_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_TARGET_STATUS") { 0 } else { 256 * VL53L5CX_NB_TARGET_PER_ZONE as usize + 4 };
const L5CX_MOT_SIZE: usize = if cfg!(feature = "VL53L5CX_DISABLE_MOTION_INDICATOR") { 0 } else { 144 };

// Macro VL53L5CX_MAX_RESULTS_SIZE indicates the maximum size used by output through I2C. 
// Value 40 corresponds to headers + meta-data + common-data and 20 corresponds to the foot
const VL53L5CX_MAX_RESULTS_SIZE: usize = 40 
    + L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE 
    + L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE 
    + L5CX_MOT_SIZE + 20;


// Macro VL53L5CX_TEMPORARY_BUFFER_SIZE can be used to know the size of the temporary buffer. 
// The minimum size is 1024, and the maximum depends of the output configuration.
pub(crate) const VL53L5CX_TEMPORARY_BUFFER_SIZE: usize = 
if VL53L5CX_MAX_RESULTS_SIZE < 1024 { 
    1024
} else {
    VL53L5CX_MAX_RESULTS_SIZE 
};

// The macro below can be used to avoid data conversion into the driver.
// By default there is a conversion between firmware and user data. Using this macro allows to use the firmware format instead of user format. The firmware format allows an increased precision.
pub(crate) const VL53L5CX_USE_RAW_FORMAT: u8 = 0;

// The macro below is used to define the number of target per zone sent through I2C. 
// This value can be changed by user, in order to tune I2C transaction, and also the total memory size (a lower number of target per zone means a lower RAM). The value must be between 1 and 4.
pub const VL53L5CX_NB_TARGET_PER_ZONE: u32 = 1;

// Define the max chunk size of the written/read data for I2C transmission
pub(crate) const I2C_CHUNK_SIZE: usize = 32;

// Macro VL53L5CX_NB_THRESHOLDS indicates the number of checkers. This value cannot be changed.
pub const VL53L5CX_NB_THRESHOLDS: usize = 64;

// Macro VL53L5CX_LAST_THRESHOLD is used to indicate the end of checkers programming.
pub const VL53L5CX_LAST_THRESHOLD: u8 = 128;

// Inner Macro for API. Not for user, only for development.
pub(crate) const VL53L5CX_DCI_DET_THRESH_CONFIG: u16 = 0x5488;
pub(crate) const VL53L5CX_DCI_DET_THRESH_GLOBAL_CONFIG: u16 = 0xB6E0;
pub(crate) const VL53L5CX_DCI_DET_THRESH_START: u16 = 0xB6E8;
pub(crate) const VL53L5CX_DCI_DET_THRESH_VALID_STATUS: u16 = 0xB9F0;

// The following macro are used to define the 'param_type' of a checker.
//  They indicate what is the measurement to catch.
pub const VL53L5CX_DISTANCE_MM: u8 = 1;
pub const VL53L5CX_SIGNAL_PER_SPAD_KCPS: u8 = 2;
pub const VL53L5CX_RANGE_SIGMA_MM: u8 = 4;
pub const VL53L5CX_AMBIENT_PER_SPAD_KCPS: u8 = 8;
pub const VL53L5CX_NB_TARGET_DETECTED: u8 = 9;
pub const VL53L5CX_TARGET_STATUS: u8 = 12;
pub const VL53L5CX_NB_SPADS_ENABLED: u8 = 13;
pub const VL53L5CX_MOTION_INDICATOR: u8 = 19;

// Inner Macro for plugin. Not for user, only for development.
pub(crate) const VL53L5CX_DCI_CAL_CFG: u16 = 0x5470;
pub(crate) const VL53L5CX_DCI_XTALK_CFG: u16 = 0xAD94;

// The following macro are used to define the 'type' of a checker.
// They indicate the window of measurements, defined by low and a high thresholds.
pub const VL53L5CX_IN_WINDOW: u8 = 0;
pub const VL53L5CX_OUT_OF_WINDOW: u8 = 1;
pub const VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER: u8 = 2;
pub const VL53L5CX_GREATER_THAN_MAX_CHECKER: u8 = 3;
pub const VL53L5CX_EQUAL_MIN_CHECKER: u8 = 4;
pub const VL53L5CX_NOT_EQUAL_MIN_CHECKER: u8 = 5;

// The following macro are used to define multiple checkers in the same zone, using operators. 
// Please note that the first checker MUST always be a OR operation.
pub const VL53L5CX_OPERATION_NONE: u8 = 0;
pub const VL53L5CX_OPERATION_OR: u8 = 0;
pub const VL53L5CX_OPERATION_AND: u8 = 2;