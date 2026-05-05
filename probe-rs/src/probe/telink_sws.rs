//! Telink SWS programmer support.

use std::{
    fmt,
    io::{Read, Write},
    net::TcpStream,
    time::Duration,
};

#[cfg(test)]
use std::any::Any;

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
const FLASH_SECTOR_SIZE: usize = 4096;
const FLASH_PAGE_SIZE: usize = 256;
const MAX_FLASH_READ_SIZE: usize = 1024;
const CMD_FUNCS: u8 = 0;
const CMD_FLASH_READ: u8 = 1;
const CMD_FLASH_WRITE: u8 = 2;
const CMD_FLASH_SECT_ERASE: u8 = 3;
const CMD_FLASH_GET_STATUS: u8 = 6;
const CMD_SWIRE_READ: u8 = 7;
const CMD_SWIRE_WRITE: u8 = 8;
const CMDF_GET_VERSION: u8 = 0;
const CMDF_SWIRE_CFG: u8 = 2;
const CMDF_EXT_POWER: u8 = 3;
const CMDF_SWIRE_ACTIVATE: u8 = 4;
const ERR_NONE: u8 = 0;

#[cfg(not(test))]
trait Io: Read + Write + Send {}

#[cfg(not(test))]
impl<T: Read + Write + Send> Io for T {}

#[cfg(test)]
trait Io: Read + Write + Send + Any {
    fn as_any(&self) -> &dyn Any;
}

#[cfg(test)]
impl<T: Read + Write + Send + Any> Io for T {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
impl dyn Io {
    fn downcast_ref_for_test<T: Any>(&self) -> Option<&T> {
        self.as_any().downcast_ref::<T>()
    }
}

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

    #[cfg(test)]
    fn from_io_for_test(io: impl Io + 'static) -> Self {
        Self {
            endpoint: "test".to_string(),
            io: Box::new(io),
            speed_khz: 960,
            attached: true,
            io_retries: 0,
        }
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

    fn response_count(response: &[u8]) -> u16 {
        u16::from_le_bytes([response[2], response[3]])
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

    fn read_flash_status(&mut self) -> Result<u8, DebugProbeError> {
        let response = self.command(&[CMD_FLASH_GET_STATUS, 0, 0, 0], 7)?;
        if Self::response_count(&response) != 1 {
            return Err(DebugProbeError::Other(
                "unexpected SWS flash status response length".to_string(),
            ));
        }
        Ok(response[4])
    }

    fn wait_flash_ready(&mut self) -> Result<(), DebugProbeError> {
        for _ in 0..300 {
            if self.read_flash_status()? & 0x01 == 0 {
                return Ok(());
            }
        }
        Err(DebugProbeError::Timeout)
    }

    fn erase_sws_flash_sector(&mut self, address: u32) -> Result<(), DebugProbeError> {
        self.command(
            &[
                CMD_FLASH_SECT_ERASE,
                address as u8,
                (address >> 8) as u8,
                (address >> 16) as u8,
            ],
            6,
        )?;
        self.wait_flash_ready()
    }

    fn program_sws_flash_page(&mut self, address: u32, data: &[u8]) -> Result<(), DebugProbeError> {
        if data.len() > FLASH_PAGE_SIZE {
            return Err(DebugProbeError::Other(format!(
                "SWS flash write block too large: {}",
                data.len()
            )));
        }

        let mut request = vec![
            CMD_FLASH_WRITE,
            address as u8,
            (address >> 8) as u8,
            (address >> 16) as u8,
        ];
        request.extend_from_slice(data);
        let response = self.command(&request, 6)?;
        if usize::from(Self::response_count(&response)) != data.len() {
            return Err(DebugProbeError::Other(format!(
                "SWS programmer wrote {} flash bytes instead of {}",
                Self::response_count(&response),
                data.len()
            )));
        }
        self.wait_flash_ready()
    }

    fn read_sws_flash_sector(
        &mut self,
        address: u32,
        data: &mut [u8],
    ) -> Result<(), DebugProbeError> {
        for (index, chunk) in data.chunks_mut(MAX_FLASH_READ_SIZE).enumerate() {
            self.read_sws_flash(address + (index * MAX_FLASH_READ_SIZE) as u32, chunk)?;
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

    fn write_sws_flash(
        &mut self,
        mut address: u32,
        mut data: &[u8],
    ) -> Result<(), DebugProbeError> {
        while !data.is_empty() {
            let sector_start = address & !((FLASH_SECTOR_SIZE as u32) - 1);
            let sector_offset = (address - sector_start) as usize;
            let chunk_len = data.len().min(FLASH_SECTOR_SIZE - sector_offset);
            let mut sector = vec![0xff; FLASH_SECTOR_SIZE];

            self.read_sws_flash_sector(sector_start, &mut sector)?;
            if sector[sector_offset..sector_offset + chunk_len] == data[..chunk_len] {
                address += chunk_len as u32;
                data = &data[chunk_len..];
                continue;
            }

            sector[sector_offset..sector_offset + chunk_len].copy_from_slice(&data[..chunk_len]);
            self.erase_sws_flash_sector(sector_start)?;

            for (page_index, page) in sector.chunks(FLASH_PAGE_SIZE).enumerate() {
                if page.iter().all(|byte| *byte == 0xff) {
                    continue;
                }
                let page_address = sector_start + (page_index * FLASH_PAGE_SIZE) as u32;
                self.program_sws_flash_page(page_address, page)?;
            }

            address += chunk_len as u32;
            data = &data[chunk_len..];
        }

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
    use std::io::{Read, Write};

    use crate::architecture::tc32::TlsrSwsDebug;

    use super::{TelinkSws, crc_block, crc_valid};

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

    #[test]
    fn flash_write_preserves_unwritten_sector_bytes() {
        let mut sector = vec![0xff; 4096];
        sector[0] = 0x11;
        sector[255] = 0x22;

        let io = MockIo::new(vec![
            response(0x01, &sector[0..1024], 1024),
            response(0x01, &sector[1024..2048], 1024),
            response(0x01, &sector[2048..3072], 1024),
            response(0x01, &sector[3072..4096], 1024),
            response(0x03, &[], 0),
            response(0x06, &[0x00], 1),
            response(0x02, &[], 256),
            response(0x06, &[0x00], 1),
        ]);
        let mut probe = TelinkSws::from_io_for_test(io);

        probe.write_sws_flash(0x1002, &[0xaa, 0xbb]).unwrap();

        let writes = probe.io.downcast_ref_for_test::<MockIo>().unwrap().writes();
        assert_eq!(
            payload_without_crc(&writes[0]),
            vec![0x01, 0x00, 0x10, 0x00, 0x00, 0x04]
        );
        assert_eq!(
            payload_without_crc(&writes[1]),
            vec![0x01, 0x00, 0x14, 0x00, 0x00, 0x04]
        );
        assert_eq!(
            payload_without_crc(&writes[2]),
            vec![0x01, 0x00, 0x18, 0x00, 0x00, 0x04]
        );
        assert_eq!(
            payload_without_crc(&writes[3]),
            vec![0x01, 0x00, 0x1c, 0x00, 0x00, 0x04]
        );
        assert_eq!(
            payload_without_crc(&writes[4]),
            vec![0x03, 0x00, 0x10, 0x00]
        );
        assert_eq!(
            payload_without_crc(&writes[5]),
            vec![0x06, 0x00, 0x00, 0x00]
        );

        let program = payload_without_crc(&writes[6]);
        assert_eq!(&program[..4], &[0x02, 0x00, 0x10, 0x00]);
        assert_eq!(program[4], 0x11);
        assert_eq!(program[4 + 2], 0xaa);
        assert_eq!(program[4 + 3], 0xbb);
        assert_eq!(program[4 + 255], 0x22);
    }

    fn response(command: u8, payload: &[u8], written_count: u16) -> Vec<u8> {
        let mut response = vec![command, 0, written_count as u8, (written_count >> 8) as u8];
        response.extend_from_slice(payload);
        crc_block(&response)
    }

    fn payload_without_crc(packet: &[u8]) -> Vec<u8> {
        packet[..packet.len() - 2].to_vec()
    }

    struct MockIo {
        reads: Vec<u8>,
        writes: Vec<Vec<u8>>,
    }

    impl MockIo {
        fn new(reads: Vec<Vec<u8>>) -> Self {
            Self {
                reads: reads.into_iter().flatten().collect(),
                writes: Vec::new(),
            }
        }

        fn writes(&self) -> &[Vec<u8>] {
            &self.writes
        }
    }

    impl Read for MockIo {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let len = buf.len().min(self.reads.len());
            buf[..len].copy_from_slice(&self.reads[..len]);
            self.reads.drain(..len);
            Ok(len)
        }
    }

    impl Write for MockIo {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.writes.push(buf.to_vec());
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
}
