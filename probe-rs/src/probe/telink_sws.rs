//! Telink SWS programmer support.

use std::{
    fmt,
    io::{Read, Write},
    net::TcpStream,
    time::Duration,
};

use serialport::SerialPort;

use crate::{
    Error,
    architecture::tc32::{Tc32CommunicationInterface, TlsrSwsDebug},
    probe::{
        DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector, ProbeCreationError,
        ProbeFactory, WireProtocol,
    },
};

const DEFAULT_UART_BAUD: u32 = 230_400;
const DEFAULT_IO_TIMEOUT: Duration = Duration::from_secs(2);
const DEFAULT_SWIRE_CONFIG: [u8; 6] = [0x5a, 0x00, 0x06, 0x02, 0x00, 0x05];
const CMD_FUNCS: u8 = 0;
const CMD_FLASH_READ: u8 = 1;
const CMD_SWIRE_READ: u8 = 7;
const CMD_SWIRE_WRITE: u8 = 8;
const CMDF_GET_VERSION: u8 = 0;
const CMDF_SWIRE_CFG: u8 = 2;
const CMDF_EXT_POWER: u8 = 3;
const CMDF_SWIRE_ACTIVATE: u8 = 4;
const ERR_NONE: u8 = 0;

trait Io: Read + Write + Send {}
impl<T: Read + Write + Send> Io for T {}

/// Factory for Telink SWS programmer endpoints.
#[derive(Debug)]
pub struct TelinkSwsFactory;

impl fmt::Display for TelinkSwsFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("TelinkSWS")
    }
}

impl ProbeFactory for TelinkSwsFactory {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        let Some(endpoint) = selector.sws_endpoint() else {
            return Err(DebugProbeError::ProbeCouldNotBeCreated(
                ProbeCreationError::NotFound,
            ));
        };
        TelinkSws::open(endpoint).map(|probe| Box::new(probe) as Box<dyn DebugProbe>)
    }

    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        Vec::new()
    }
}

/// Telink SWS programmer debug probe.
pub struct TelinkSws {
    endpoint: String,
    io: Box<dyn Io>,
    speed_khz: u32,
    attached: bool,
    io_retries: usize,
}

impl fmt::Debug for TelinkSws {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TelinkSws")
            .field("endpoint", &self.endpoint)
            .field("speed_khz", &self.speed_khz)
            .field("attached", &self.attached)
            .finish()
    }
}

impl TelinkSws {
    /// Open a Telink SWS programmer endpoint.
    pub fn open(endpoint: &str) -> Result<Self, DebugProbeError> {
        let io: Box<dyn Io> = if let Some(tcp_endpoint) = endpoint.strip_prefix("tcp://") {
            let stream = TcpStream::connect(tcp_endpoint).map_err(DebugProbeError::Usb)?;
            stream
                .set_read_timeout(Some(DEFAULT_IO_TIMEOUT))
                .map_err(DebugProbeError::Usb)?;
            stream
                .set_write_timeout(Some(DEFAULT_IO_TIMEOUT))
                .map_err(DebugProbeError::Usb)?;
            Box::new(stream)
        } else {
            let port = serialport::new(endpoint, DEFAULT_UART_BAUD)
                .dtr_on_open(false)
                .timeout(DEFAULT_IO_TIMEOUT)
                .open()
                .map_err(|_| {
                    DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::CouldNotOpen)
                })?;
            Box::new(SerialIo(port))
        };

        Ok(Self {
            endpoint: endpoint.to_string(),
            io,
            speed_khz: 960,
            attached: false,
            io_retries: 3,
        })
    }

    fn command(&mut self, request: &[u8], expected_len: usize) -> Result<Vec<u8>, DebugProbeError> {
        let mut last_error = None;
        for _ in 0..=self.io_retries {
            match self.command_once(request, expected_len) {
                Ok(response) => return Ok(response),
                Err(error) => last_error = Some(error),
            }
        }
        Err(last_error.unwrap_or_else(|| DebugProbeError::Other("SWS command failed".to_string())))
    }

    fn command_once(
        &mut self,
        request: &[u8],
        expected_len: usize,
    ) -> Result<Vec<u8>, DebugProbeError> {
        let packet = crc_block(request);
        self.io.write_all(&packet).map_err(DebugProbeError::Usb)?;
        self.io.flush().map_err(DebugProbeError::Usb)?;

        let mut response = vec![0u8; expected_len];
        self.io
            .read_exact(&mut response)
            .map_err(|error| match error.kind() {
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock => {
                    DebugProbeError::Timeout
                }
                _ => DebugProbeError::Usb(error),
            })?;

        if !crc_valid(&response) {
            return Err(DebugProbeError::Other(
                "invalid SWS programmer CRC".to_string(),
            ));
        }
        if response.first().copied() != request.first().copied() {
            return Err(DebugProbeError::Other(
                "unexpected SWS programmer response command".to_string(),
            ));
        }
        if response.get(1).copied() != Some(ERR_NONE) {
            return Err(DebugProbeError::Other(format!(
                "SWS programmer returned error {}",
                response.get(1).copied().unwrap_or(0xff)
            )));
        }
        Ok(response)
    }

    fn get_version(&mut self) -> Result<Vec<u8>, DebugProbeError> {
        self.command(&[CMD_FUNCS, CMDF_GET_VERSION, 0, 0], 19)
    }

    fn set_swire_config(&mut self, swdiv: u8, swaddrlen: u8) -> Result<(), DebugProbeError> {
        let mut request = vec![CMD_FUNCS, CMDF_SWIRE_CFG, 0, 0, swdiv, swaddrlen];
        request.extend_from_slice(&DEFAULT_SWIRE_CONFIG);
        self.command(&request, 14)?;
        self.speed_khz = (24_000 / 5 / u32::from(swdiv)).max(1);
        Ok(())
    }

    fn set_reset_pin(&mut self, enabled: bool) -> Result<(), DebugProbeError> {
        self.command(
            &[CMD_FUNCS, CMDF_EXT_POWER, if enabled { 1 } else { 0 }, 0],
            7,
        )?;
        Ok(())
    }

    fn activate(
        &mut self,
        activate_ms: u16,
        swdiv: u8,
        swaddrlen: u8,
    ) -> Result<(), DebugProbeError> {
        let count = if activate_ms > 0 {
            let bit_time = (f32::from(swaddrlen) + 5.7) * 5.0 * f32::from(swdiv) / 2400.0;
            (f32::from(activate_ms) / bit_time).min(f32::from(u16::MAX)) as u16
        } else {
            0
        };
        let request = [
            CMD_FUNCS,
            CMDF_SWIRE_ACTIVATE,
            count as u8,
            (count >> 8) as u8,
        ];
        let packet = crc_block(&request);
        self.io.write_all(&packet).map_err(DebugProbeError::Usb)?;
        self.io.flush().map_err(DebugProbeError::Usb)?;

        let mut header = [0u8; 6];
        self.io
            .read_exact(&mut header)
            .map_err(DebugProbeError::Usb)?;
        let payload_len = u16::from_le_bytes([header[2], header[3]]) as usize;
        let mut response = header.to_vec();
        if payload_len > 0 {
            let mut payload = vec![0u8; payload_len];
            self.io
                .read_exact(&mut payload)
                .map_err(DebugProbeError::Usb)?;
            response.extend_from_slice(&payload);
        }
        if !crc_valid(&response) {
            return Err(DebugProbeError::Other(
                "invalid SWS activate CRC".to_string(),
            ));
        }
        if response[1] != ERR_NONE {
            return Err(DebugProbeError::Other(format!(
                "SWS activate failed with error {}",
                response[1]
            )));
        }
        Ok(())
    }
}

impl DebugProbe for TelinkSws {
    fn get_name(&self) -> &str {
        "Telink SWS Programmer"
    }

    fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let swdiv = (24_000 / 5 / speed_khz.max(1)).clamp(1, u32::from(u8::MAX)) as u8;
        self.set_swire_config(swdiv, 3)?;
        Ok(self.speed_khz)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        let version = self.get_version()?;
        let programmer_clock_mhz = if version.get(4..6) == Some(&[0x62, 0x55]) {
            24
        } else {
            24
        };
        let swdiv = (programmer_clock_mhz / 5).max(1) as u8;
        self.set_swire_config(swdiv, 3)?;
        self.set_reset_pin(false)?;
        self.activate(70, swdiv, 3)?;
        self.attached = true;
        Ok(())
    }

    fn detach(&mut self) -> Result<(), Error> {
        self.attached = false;
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.set_reset_pin(true)?;
        self.set_reset_pin(false)
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        self.set_reset_pin(true)
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        self.set_reset_pin(false)
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        match protocol {
            WireProtocol::Swd => Ok(()),
            _ => Err(DebugProbeError::UnsupportedProtocol(protocol)),
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        Some(WireProtocol::Swd)
    }

    fn has_tc32_interface(&self) -> bool {
        true
    }

    fn try_get_tc32_interface<'probe>(
        &'probe mut self,
    ) -> Result<Tc32CommunicationInterface<'probe>, DebugProbeError> {
        Ok(Tc32CommunicationInterface::new(self))
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }
}

impl TlsrSwsDebug for TelinkSws {
    fn read_sws_memory(&mut self, address: u32, data: &mut [u8]) -> Result<(), DebugProbeError> {
        let request = [
            CMD_SWIRE_READ,
            address as u8,
            (address >> 8) as u8,
            (address >> 16) as u8,
            data.len() as u8,
            (data.len() >> 8) as u8,
        ];
        let response = self.command(&request, data.len() + 6)?;
        data.copy_from_slice(&response[4..4 + data.len()]);
        Ok(())
    }

    fn read_sws_flash(&mut self, address: u32, data: &mut [u8]) -> Result<(), DebugProbeError> {
        let request = [
            CMD_FLASH_READ,
            address as u8,
            (address >> 8) as u8,
            (address >> 16) as u8,
            data.len() as u8,
            (data.len() >> 8) as u8,
        ];
        let response = self.command(&request, data.len() + 6)?;
        data.copy_from_slice(&response[4..4 + data.len()]);
        Ok(())
    }

    fn write_sws_memory(&mut self, address: u32, data: &[u8]) -> Result<(), DebugProbeError> {
        let mut request = vec![
            CMD_SWIRE_WRITE,
            address as u8,
            (address >> 8) as u8,
            (address >> 16) as u8,
        ];
        request.extend_from_slice(data);
        self.command(&request, 6)?;
        Ok(())
    }
}

struct SerialIo(Box<dyn SerialPort>);

impl Read for SerialIo {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

impl Write for SerialIo {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.0.flush()
    }
}

fn crc16(data: &[u8]) -> [u8; 2] {
    let mut crc = 0xffffu16;
    for value in data {
        crc ^= u16::from(*value);
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xa001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc.to_le_bytes()
}

fn crc_block(data: &[u8]) -> Vec<u8> {
    let mut framed = Vec::with_capacity(data.len() + 2);
    framed.extend_from_slice(data);
    framed.extend_from_slice(&crc16(data));
    framed
}

fn crc_valid(data: &[u8]) -> bool {
    data.len() >= 2 && crc16(&data[..data.len() - 2]) == data[data.len() - 2..]
}

#[cfg(test)]
mod tests {
    use super::{crc_block, crc_valid};

    #[test]
    fn crc_block_matches_tlsr_programmer_get_version_packet() {
        assert_eq!(
            crc_block(&[0x00, 0x00, 0x00, 0x00]),
            vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x24]
        );
    }

    #[test]
    fn crc_valid_accepts_framed_packet_and_rejects_corruption() {
        let packet = crc_block(&[0x07, 0xbc, 0x06, 0x00, 0x04, 0x00]);

        assert!(crc_valid(&packet));

        let mut corrupted = packet;
        corrupted[2] ^= 0x01;
        assert!(!crc_valid(&corrupted));
    }
}
