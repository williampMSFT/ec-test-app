use crate::{RtcSource, Source, Threshold, common};
use color_eyre::{Result, eyre::eyre};
use std::ffi;
use time_alarm_service_messages::{
    AcpiTimerId, AcpiTimestamp, AlarmExpiredWakePolicy, AlarmTimerSeconds, TimeAlarmDeviceCapabilities, TimerStatus,
};

// This module maps the data returned from call into the C-Library to RUST structures
unsafe extern "C" {
    fn EvaluateAcpi(input: *const i8, input_len: usize, buffer: *mut u8, buf_len: &mut usize) -> i32;
}

const ERROR_SUCCESS: i32 = 0;

mod guid {
    pub const _SENSOR_CRT_TEMP: uuid::Uuid = uuid::uuid!("218246e7-baf6-45f1-aa13-07e4845256b8");
    pub const _SENSOR_PROCHOT_TEMP: uuid::Uuid = uuid::uuid!("22dc52d2-fd0b-47ab-95b8-26552f9831a5");
    pub const FAN_ON_TEMP: uuid::Uuid = uuid::uuid!("ba17b567-c368-48d5-bc6f-a312a41583c1");
    pub const FAN_RAMP_TEMP: uuid::Uuid = uuid::uuid!("3a62688c-d95b-4d2d-bacc-90d7a5816bcd");
    pub const FAN_MAX_TEMP: uuid::Uuid = uuid::uuid!("dcb758b1-f0fd-4ec7-b2c0-ef1e2a547b76");
    pub const FAN_MIN_RPM: uuid::Uuid = uuid::uuid!("db261c77-934b-45e2-9742-256c62badb7a");
    pub const FAN_MAX_RPM: uuid::Uuid = uuid::uuid!("5cf839df-8be7-42b9-9ac5-3403ca2c8a6a");
    pub const FAN_CURRENT_RPM: uuid::Uuid = uuid::uuid!("adf95492-0776-4ffc-84f3-b6c8b5269683");
}

mod serialization {
    use super::*;

    const ACPI_EVAL_INPUT_BUFFER_COMPLEX_SIGNATURE_EX: u32 = u32::from_le_bytes(*b"AeiF");
    const ACPI_MAX_METHOD_NAME_LEN: usize = 256;

    impl From<num_enum::TryFromPrimitiveError<AcpiArgumentType>> for AcpiParseError {
        fn from(_: num_enum::TryFromPrimitiveError<AcpiArgumentType>) -> Self {
            AcpiParseError::InvalidFormat
        }
    }

    #[derive(Debug)]
    pub enum AcpiValue {
        Integer(u32),
        String(String),
        Buffer(Vec<u8>),
        Package(Box<Vec<AcpiValue>>),
    }

    impl AcpiValue {
        pub fn from_guid(guid: uuid::Uuid) -> Self {
            AcpiValue::Buffer(guid.to_bytes_le().to_vec())
        }
    }

    #[derive(num_enum::IntoPrimitive, num_enum::TryFromPrimitive, Debug, Copy, Clone, Eq, PartialEq)]
    #[repr(u16)]
    /// ACPI argument types - these correspond to the ACPI_METHOD_ARGUMENT_* defines in apiioct.h from the Windows SDK
    enum AcpiArgumentType {
        Integer = 0x0,
        String = 0x1,
        Buffer = 0x2,
        Package = 0x3,
    }

    impl AcpiValue {
        fn serialize(&self) -> Vec<u8> {
            match self {
                AcpiValue::Integer(i) => {
                    let header = AcpiMethodArgumentV1Header {
                        type_: AcpiArgumentType::Integer.into(),
                        data_length: core::mem::size_of::<u32>() as u16,
                    };
                    let mut buf = Vec::new();
                    buf.extend(bytemuck::bytes_of(&header));
                    buf.extend(&i.to_le_bytes());
                    buf
                }
                AcpiValue::String(s) => {
                    let cstr = ffi::CString::new(s.as_str()).expect("String contained null byte");
                    let cstr_bytes = cstr.as_bytes_with_nul();
                    let header = AcpiMethodArgumentV1Header {
                        type_: AcpiArgumentType::String.into(),
                        data_length: cstr_bytes.len() as u16,
                    };
                    let mut buf = Vec::new();
                    buf.extend(bytemuck::bytes_of(&header));
                    buf.extend(cstr_bytes);
                    buf
                }
                AcpiValue::Buffer(b) => {
                    let header = AcpiMethodArgumentV1Header {
                        type_: AcpiArgumentType::Buffer.into(),
                        data_length: b.len() as u16,
                    };
                    let mut buf = Vec::new();
                    buf.extend(bytemuck::bytes_of(&header));
                    buf.extend(b);
                    buf
                }
                AcpiValue::Package(elements) => {
                    let mut element_buffer = Vec::new();
                    for element in elements.iter() {
                        element_buffer.extend(element.serialize());
                    }

                    let header = AcpiMethodArgumentV1Header {
                        type_: AcpiArgumentType::Package.into(),
                        data_length: element_buffer.len() as u16,
                    };

                    let mut result = Vec::new();
                    result.extend(bytemuck::bytes_of(&header));
                    result.extend(element_buffer);
                    result
                }
            }
        }

        fn deserialize(data: &[u8]) -> Result<(Self, &[u8]), AcpiParseError> {
            let (header, payload) = data.split_at(core::mem::size_of::<AcpiMethodArgumentV1Header>());
            let header = bytemuck::try_from_bytes::<AcpiMethodArgumentV1Header>(header)
                .map_err(|_| AcpiParseError::InvalidFormat)?;

            if payload.len() < header.data_length as usize {
                return Err(AcpiParseError::InsufficientLength);
            }

            match AcpiArgumentType::try_from(header.type_)? {
                AcpiArgumentType::Integer => {
                    if header.data_length as usize != core::mem::size_of::<u32>() {
                        return Err(AcpiParseError::InvalidFormat);
                    }
                    let (payload, remaining) = payload.split_at(core::mem::size_of::<u32>());
                    let int_bytes: [u8; 4] = payload[0..4].try_into().map_err(|_| AcpiParseError::InvalidFormat)?;
                    Ok((AcpiValue::Integer(u32::from_le_bytes(int_bytes)), remaining))
                }
                AcpiArgumentType::String => {
                    let (payload, remaining) = payload.split_at(header.data_length as usize);
                    let string = cstr_bytes_to_string(payload).map_err(|_| AcpiParseError::InvalidFormat)?;
                    Ok((AcpiValue::String(string), remaining))
                }
                AcpiArgumentType::Buffer => {
                    let (payload, remaining) = payload.split_at(header.data_length as usize);
                    let buffer = payload.to_vec();
                    Ok((AcpiValue::Buffer(buffer), remaining))
                }
                AcpiArgumentType::Package => {
                    let mut elements = Vec::new();
                    let mut remaining = payload;
                    while !remaining.is_empty() {
                        let (element, rest) = AcpiValue::deserialize(remaining)?;
                        elements.push(element);
                        remaining = rest;
                    }
                    Ok((AcpiValue::Package(Box::new(elements)), remaining))
                }
            }
        }
    }

    fn cstr_bytes_to_string(raw: &[u8]) -> Result<String> {
        Ok(ffi::CStr::from_bytes_until_nul(raw)
            .map_err(|_| color_eyre::eyre::eyre!("Invalid byte slice"))?
            .to_str()
            .map_err(|_| color_eyre::eyre::eyre!("String contains invalid characters"))?
            .to_owned())
    }

    #[repr(C)]
    #[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
    struct AcpiMethodArgumentV1Header {
        type_: u16,
        data_length: u16, // Followed by either 4 bytes of data (for integers) or a variable-length buffer (for strings/buffers/packages)
    }

    #[repr(C)]
    #[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
    struct AcpiEvalOutputBufferV1Header {
        _signature: u32,
        length: u32,
        count: u32, // Followed by `count` number of AcpiMethodArgumentV1Header + data
    }

    #[repr(C)]
    #[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
    struct AcpiEvalInputBufferComplexV1ExHeader {
        signature: u32, // must be ACPI_EVAL_INPUT_BUFFER_COMPLEX_SIGNATURE_EX
        method_name: [u8; ACPI_MAX_METHOD_NAME_LEN],
        size: u32,
        argument_count: u32,
        // Followed by any number of AcpiMethodArgumentV1Header+data structs
    }

    pub fn serialize_acpi_request(name: &str, args: &[AcpiValue]) -> Result<Vec<u8>, AcpiParseError> {
        // Maximum number of arguments allowed is 7 as per spec
        if args.len() > 7 {
            return Err(AcpiParseError::InsufficientLength);
        }

        if name.len() > ACPI_MAX_METHOD_NAME_LEN {
            return Err(AcpiParseError::InsufficientLength);
        }

        let mut args_buffer = Vec::new();
        for arg in args.iter() {
            args_buffer.extend(arg.serialize());
        }

        let header = AcpiEvalInputBufferComplexV1ExHeader {
            signature: ACPI_EVAL_INPUT_BUFFER_COMPLEX_SIGNATURE_EX,
            method_name: {
                let mut buffer = [0u8; ACPI_MAX_METHOD_NAME_LEN];
                let bytes = name.as_bytes();
                buffer[..bytes.len()].copy_from_slice(bytes);
                buffer
            },
            size: args_buffer.len() as u32,
            argument_count: args.len() as u32,
        };

        let mut result = Vec::new();
        result.extend(bytemuck::bytes_of(&header));
        result.extend(args_buffer);
        Ok(result)
    }

    pub fn deserialize_acpi_response(data: &[u8]) -> Result<Vec<AcpiValue>, AcpiParseError> {
        let (header, payload) = data.split_at(core::mem::size_of::<AcpiEvalOutputBufferV1Header>());
        let header = bytemuck::try_from_bytes::<AcpiEvalOutputBufferV1Header>(header)
            .map_err(|_| AcpiParseError::InvalidFormat)?;

        let mut result = Vec::new();
        let payload = payload
            .get(..header.length as usize)
            .ok_or(AcpiParseError::InsufficientLength)?;
        let mut remaining = payload;
        for _ in 0..header.count {
            let (value, rest) = AcpiValue::deserialize(remaining)?;
            result.push(value);
            remaining = rest;
        }

        Ok(result)
    }
}

use serialization::AcpiValue;

#[derive(Debug)]
pub enum AcpiParseError {
    InsufficientLength,
    InvalidFormat,
    EvaluationFailed(i32),
}

impl std::error::Error for AcpiParseError {}
impl std::fmt::Display for AcpiParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

#[derive(Default, Copy, Clone)]
pub struct Acpi {}

impl Acpi {
    pub fn new() -> Self {
        Default::default()
    }

    fn evaluate(name: &str, args: &[AcpiValue]) -> Result<Vec<AcpiValue>, AcpiParseError> {
        // Input buffer
        let in_buf: Vec<u8> = serialization::serialize_acpi_request(name, args)?;
        let in_buf_len = in_buf.len();

        // Output buffer
        let mut out_buf_len = 1024;
        let mut out_buf = vec![0u8; out_buf_len];

        let res = unsafe {
            EvaluateAcpi(
                in_buf.as_ptr() as *const i8,
                in_buf_len,
                out_buf.as_mut_ptr(),
                &mut out_buf_len,
            )
        };

        match res {
            ERROR_SUCCESS => serialization::deserialize_acpi_response(&out_buf),
            err => Err(AcpiParseError::EvaluationFailed(err)),
        }
    }

    /// Evaluates the provided method with the provided arguments and returns its single u32 result.
    /// Errors if the result is not a single u32.
    fn evaluate_u32(name: &str, args: &[AcpiValue]) -> Result<u32> {
        let output = Acpi::evaluate(name, args)?;

        if output.len() != 1 {
            Err(eyre!(
                "{} returned unexpected number of arguments: {}",
                name,
                output.len()
            ))
        } else if let AcpiValue::Integer(i) = output[0] {
            Ok(i)
        } else {
            Err(eyre!("{} did not return an integer as expected", name))
        }
    }
}

fn acpi_get_var(guid: uuid::Uuid) -> Result<f64> {
    let output = Acpi::evaluate("\\_SB.ECT0.TGVR", &[AcpiValue::Integer(1), AcpiValue::from_guid(guid)])?;

    if let [AcpiValue::Integer(error_code), AcpiValue::Integer(value)] = output.as_slice() {
        if *error_code == 0 {
            Ok(f64::from(*value))
        } else {
            Err(eyre!("GET_VAR({guid}) returned error code {error_code}"))
        }
    } else {
        Err(eyre!("GET_VAR({guid}) unrecognized output - got {:?}", output))
    }
}

fn acpi_set_var(guid: uuid::Uuid, value: f64) -> Result<()> {
    let result_code = Acpi::evaluate_u32(
        "\\_SB.ECT0.TSVR",
        &[
            AcpiValue::Integer(1),
            AcpiValue::from_guid(guid),
            AcpiValue::Integer(value as u32),
        ],
    )?;

    if result_code == 0 {
        Ok(())
    } else {
        Err(eyre!("SET_VAR({guid}, {value}) returned error code {result_code}"))
    }
}

impl Source for Acpi {
    fn get_temperature(&self) -> Result<f64> {
        let output = Acpi::evaluate_u32("\\_SB.ECT0.RTMP", &[])?;
        Ok(common::dk_to_c(output))
    }

    fn get_rpm(&self) -> Result<f64> {
        acpi_get_var(guid::FAN_CURRENT_RPM)
    }

    fn get_min_rpm(&self) -> Result<f64> {
        acpi_get_var(guid::FAN_MIN_RPM)
    }

    fn get_max_rpm(&self) -> Result<f64> {
        acpi_get_var(guid::FAN_MAX_RPM)
    }

    fn get_threshold(&self, threshold: Threshold) -> Result<f64> {
        match threshold {
            Threshold::On => Ok(common::dk_to_c(acpi_get_var(guid::FAN_ON_TEMP)? as u32)),
            Threshold::Ramping => Ok(common::dk_to_c(acpi_get_var(guid::FAN_RAMP_TEMP)? as u32)),
            Threshold::Max => Ok(common::dk_to_c(acpi_get_var(guid::FAN_MAX_TEMP)? as u32)),
        }
    }

    fn set_rpm(&self, rpm: f64) -> Result<()> {
        acpi_set_var(guid::FAN_CURRENT_RPM, rpm)
    }

    fn get_bst(&self) -> Result<crate::battery::BstData> {
        let data = Acpi::evaluate("\\_SB.ECT0.TBST", &[])?;

        if let [
            AcpiValue::Integer(state),
            AcpiValue::Integer(rate),
            AcpiValue::Integer(capacity),
            AcpiValue::Integer(voltage),
        ] = data.as_slice()
        {
            Ok(crate::battery::BstData {
                state: crate::battery::ChargeState::try_from(*state)?,
                rate: *rate,
                capacity: *capacity,
                voltage: *voltage,
            })
        } else {
            Err(eyre!("GET_BST unrecognized output"))
        }
    }

    fn get_bix(&self) -> Result<crate::battery::BixData> {
        // TODO this looks odd to me. Spec allows for only 7 input args. Not sure what the max outputs are, but 21 seems high and it looks
        //      like the spec expects a package output with 21 elements, but this code is expecting 21 top-level arguments.
        //      I wonder if a single package-return-value is auto unwrapped by the ACPI driver or something?
        //      If it is, there may be some other issue with get_var...
        let data = Acpi::evaluate("\\_SB.ECT0.TBIX", &[])?;
        if let [
            AcpiValue::Integer(revision),
            AcpiValue::Integer(power_unit),
            AcpiValue::Integer(design_capacity),
            AcpiValue::Integer(last_full_capacity),
            AcpiValue::Integer(battery_technology),
            AcpiValue::Integer(design_voltage),
            AcpiValue::Integer(warning_capacity),
            AcpiValue::Integer(low_capacity),
            AcpiValue::Integer(cycle_count),
            AcpiValue::Integer(accuracy),
            AcpiValue::Integer(max_sample_time),
            AcpiValue::Integer(min_sample_time),
            AcpiValue::Integer(max_average_interval),
            AcpiValue::Integer(min_average_interval),
            AcpiValue::Integer(capacity_gran1),
            AcpiValue::Integer(capacity_gran2),
            AcpiValue::String(model_number),
            AcpiValue::String(serial_number),
            AcpiValue::String(battery_type),
            AcpiValue::String(oem_info),
            AcpiValue::Integer(swap_cap),
        ] = data.as_slice()
        {
            Ok(crate::battery::BixData {
                revision: *revision,
                power_unit: crate::battery::PowerUnit::try_from(*power_unit)?,
                design_capacity: *design_capacity,
                last_full_capacity: *last_full_capacity,
                battery_technology: crate::battery::BatteryTechnology::try_from(*battery_technology)?,
                design_voltage: *design_voltage,
                warning_capacity: *warning_capacity,
                low_capacity: *low_capacity,
                cycle_count: *cycle_count,
                accuracy: *accuracy,
                max_sample_time: *max_sample_time,
                min_sample_time: *min_sample_time,
                max_average_interval: *max_average_interval,
                min_average_interval: *min_average_interval,
                capacity_gran1: *capacity_gran1,
                capacity_gran2: *capacity_gran2,
                model_number: model_number.clone(),
                serial_number: serial_number.clone(),
                battery_type: battery_type.clone(),
                oem_info: oem_info.clone(),
                swap_cap: crate::battery::SwapCap::try_from(*swap_cap)?,
            })
        } else {
            Err(eyre!("GET_BIX unrecognized output - got {:?}", data))
        }
    }

    fn set_btp(&self, trippoint: u32) -> Result<()> {
        // No return value is expected according to ACPI spec
        let _ = Acpi::evaluate("\\_SB.ECT0.TBTP", &[AcpiValue::Integer(trippoint)])?;
        Ok(())
    }
}

impl RtcSource for Acpi {
    fn get_capabilities(&self) -> Result<TimeAlarmDeviceCapabilities> {
        Ok(TimeAlarmDeviceCapabilities(Acpi::evaluate_u32("\\_SB.ECT0._GCP", &[])?))
    }

    fn get_real_time(&self) -> Result<AcpiTimestamp> {
        let result = Acpi::evaluate("\\_SB.ECT0._GRT", &[])?;

        if let [AcpiValue::Buffer(buffer)] = result.as_slice() {
            AcpiTimestamp::try_from_bytes(buffer)
                .map_err(|e| eyre!("GET_REAL_TIME invalid output format: {:?} for bytes {:?}", e, buffer))
        } else {
            Err(eyre!("GET_REAL_TIME invalid output type {:?}", result))
        }
    }

    fn get_wake_status(&self, timer_id: AcpiTimerId) -> Result<TimerStatus> {
        Ok(TimerStatus(Acpi::evaluate_u32(
            "\\_SB.ECT0._GWS",
            &[AcpiValue::Integer(timer_id.into())],
        )?))
    }

    fn get_expired_timer_wake_policy(&self, timer_id: AcpiTimerId) -> Result<AlarmExpiredWakePolicy> {
        Ok(AlarmExpiredWakePolicy(Acpi::evaluate_u32(
            "\\_SB.ECT0._TIP",
            &[AcpiValue::Integer(timer_id.into())],
        )?))
    }

    fn get_timer_value(&self, timer_id: AcpiTimerId) -> Result<AlarmTimerSeconds> {
        Ok(AlarmTimerSeconds(Acpi::evaluate_u32(
            "\\_SB.ECT0._TIV",
            &[AcpiValue::Integer(timer_id.into())],
        )?))
    }
}
