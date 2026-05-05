//! Telink TC32 architecture support.

use std::time::{Duration, Instant};

use crate::{
    BreakpointCause, CoreInformation, CoreInterface, CoreRegister, CoreRegisters, CoreStatus,
    CoreType, Error, HaltReason, InstructionSet, MemoryInterface, RegisterId, RegisterValue,
    memory::MemoryNotAlignedError, probe::DebugProbeError,
};

pub mod registers;

const FLASH_ADDRESS_MAX: u64 = 0x80000;
const REG_DEBUG_CONTROL: u32 = 0x602;
const REG_DEBUG_STEP: u32 = 0x613;
const REG_BREAKPOINTS: u32 = 0x610;
const REG_SNAPSHOT: u32 = 0x680;
const REG_PC: u32 = 0x6bc;
const BREAKPOINT_PHYSICAL_SLOTS: usize = 4;
const BREAKPOINT_LOGICAL_LIMIT: usize = 3;
const BREAKPOINT_PROGRAM_SETTLE: Duration = Duration::from_millis(2);

/// SWS operations required by the TC32 core implementation.
pub trait TlsrSwsDebug {
    /// Read SRAM/register-mapped memory over SWS.
    fn read_sws_memory(&mut self, address: u32, data: &mut [u8]) -> Result<(), DebugProbeError>;

    /// Read flash over the programmer's flash-read command.
    fn read_sws_flash(&mut self, address: u32, data: &mut [u8]) -> Result<(), DebugProbeError>;

    /// Write SRAM/register-mapped memory over SWS.
    fn write_sws_memory(&mut self, address: u32, data: &[u8]) -> Result<(), DebugProbeError>;
}

/// TC32 debug communication interface.
pub struct Tc32CommunicationInterface<'probe> {
    sws: &'probe mut dyn TlsrSwsDebug,
}

impl<'probe> Tc32CommunicationInterface<'probe> {
    /// Create a new TC32 communication interface.
    pub fn new(sws: &'probe mut dyn TlsrSwsDebug) -> Self {
        Self { sws }
    }

    fn read_memory(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        let mut offset = 0usize;
        while offset < data.len() {
            let chunk_len = (data.len() - offset).min(1024);
            let chunk_addr = address
                .checked_add(offset as u64)
                .ok_or_else(|| Error::Other("TC32 memory address overflow".to_string()))?;
            let chunk_addr = u32::try_from(chunk_addr)
                .map_err(|_| Error::Other(format!("TC32 address {chunk_addr:#x} out of range")))?;
            let chunk = &mut data[offset..offset + chunk_len];
            if u64::from(chunk_addr) < FLASH_ADDRESS_MAX {
                self.sws.read_sws_flash(chunk_addr, chunk)?;
            } else {
                self.sws.read_sws_memory(chunk_addr, chunk)?;
            }
            offset += chunk_len;
        }
        Ok(())
    }

    fn write_memory(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        let address = u32::try_from(address)
            .map_err(|_| Error::Other(format!("TC32 address {address:#x} out of range")))?;
        self.sws.write_sws_memory(address, data)?;
        Ok(())
    }

    fn read_pc(&mut self) -> Result<u32, Error> {
        let mut bytes = [0u8; 4];
        self.sws.read_sws_memory(REG_PC, &mut bytes)?;
        Ok(u32::from_le_bytes(bytes))
    }
}

/// Cached TC32 core state.
#[derive(Debug)]
pub struct Tc32CoreState {
    pub(crate) hardware_breakpoints: [Option<u64>; 3],
    pub(crate) breakpoints_enabled: bool,
    pub(crate) status: CoreStatus,
}

impl Tc32CoreState {
    pub(crate) fn new() -> Self {
        Self {
            hardware_breakpoints: [None; 3],
            breakpoints_enabled: true,
            status: CoreStatus::Unknown,
        }
    }
}

/// A TC32 core accessed through Telink SWS debug registers.
pub struct Tc32<'probe> {
    interface: Tc32CommunicationInterface<'probe>,
    state: &'probe mut Tc32CoreState,
}

impl<'probe> Tc32<'probe> {
    /// Create a TC32 core wrapper.
    pub(crate) fn new(
        interface: Tc32CommunicationInterface<'probe>,
        state: &'probe mut Tc32CoreState,
    ) -> Self {
        Self { interface, state }
    }

    fn write_debug_control(&mut self, value: u8) -> Result<(), Error> {
        self.interface
            .write_memory(u64::from(REG_DEBUG_CONTROL), &[value])
    }

    fn write_breakpoint_payload(&mut self) -> Result<(), Error> {
        let mut encoded = [0u32; BREAKPOINT_PHYSICAL_SLOTS];
        if self.state.breakpoints_enabled {
            for (slot, address) in self.state.hardware_breakpoints.iter().enumerate() {
                if let Some(address) = address {
                    encoded[slot + 1] = (*address as u32) | 0x0100_0000;
                }
            }
        }

        let mut payload = [0u8; BREAKPOINT_PHYSICAL_SLOTS * 4];
        for (slot, value) in encoded.iter().enumerate() {
            payload[slot * 4..slot * 4 + 4].copy_from_slice(&value.to_le_bytes());
        }
        self.interface
            .write_memory(u64::from(REG_BREAKPOINTS), &payload)?;
        std::thread::sleep(BREAKPOINT_PROGRAM_SETTLE);
        Ok(())
    }

    fn active_breakpoints(&self) -> impl Iterator<Item = u64> + '_ {
        self.state
            .hardware_breakpoints
            .iter()
            .flatten()
            .copied()
            .filter(|_| self.state.breakpoints_enabled)
    }

    fn has_active_breakpoints(&self) -> bool {
        self.active_breakpoints().next().is_some()
    }

    fn read_register_snapshot(&mut self) -> Result<[u32; 32], Error> {
        let mut payload = [0u8; 128];
        self.interface.sws.read_sws_memory(REG_SNAPSHOT, &mut payload)?;
        let mut snapshot = [0u32; 32];
        for (index, chunk) in payload.chunks_exact(4).enumerate() {
            snapshot[index] = u32::from_le_bytes(chunk.try_into().unwrap());
        }
        snapshot[15] = self.interface.read_pc()?;
        Ok(snapshot)
    }
}

impl MemoryInterface for Tc32<'_> {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        if !address.is_multiple_of(8) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 8,
            }
            .into());
        }
        let mut bytes = vec![0u8; data.len() * 8];
        self.read_8(address, &mut bytes)?;
        for (value, chunk) in data.iter_mut().zip(bytes.chunks_exact(8)) {
            *value = u64::from_le_bytes(chunk.try_into().unwrap());
        }
        Ok(())
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        if !address.is_multiple_of(4) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 4,
            }
            .into());
        }
        let mut bytes = vec![0u8; data.len() * 4];
        self.read_8(address, &mut bytes)?;
        for (value, chunk) in data.iter_mut().zip(bytes.chunks_exact(4)) {
            *value = u32::from_le_bytes(chunk.try_into().unwrap());
        }
        Ok(())
    }

    fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
        if !address.is_multiple_of(2) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 2,
            }
            .into());
        }
        let mut bytes = vec![0u8; data.len() * 2];
        self.read_8(address, &mut bytes)?;
        for (value, chunk) in data.iter_mut().zip(bytes.chunks_exact(2)) {
            *value = u16::from_le_bytes(chunk.try_into().unwrap());
        }
        Ok(())
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        self.interface.read_memory(address, data)
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), Error> {
        if !address.is_multiple_of(8) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 8,
            }
            .into());
        }
        let mut bytes = Vec::with_capacity(data.len() * 8);
        for value in data {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        self.write_8(address, &bytes)
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        if !address.is_multiple_of(4) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 4,
            }
            .into());
        }
        let mut bytes = Vec::with_capacity(data.len() * 4);
        for value in data {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        self.write_8(address, &bytes)
    }

    fn write_16(&mut self, address: u64, data: &[u16]) -> Result<(), Error> {
        if !address.is_multiple_of(2) {
            return Err(MemoryNotAlignedError {
                address,
                alignment: 2,
            }
            .into());
        }
        let mut bytes = Vec::with_capacity(data.len() * 2);
        for value in data {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        self.write_8(address, &bytes)
    }

    fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        self.interface.write_memory(address, data)
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        Ok(true)
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

impl CoreInterface for Tc32<'_> {
    fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), Error> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.core_halted()? {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    fn core_halted(&mut self) -> Result<bool, Error> {
        Ok(matches!(self.state.status, CoreStatus::Halted(_)))
    }

    fn status(&mut self) -> Result<CoreStatus, Error> {
        if matches!(self.state.status, CoreStatus::Running) {
            let pc = u64::from(self.interface.read_pc()?);
            if self.active_breakpoints().any(|address| address == pc) {
                self.state.status =
                    CoreStatus::Halted(HaltReason::Breakpoint(BreakpointCause::Hardware));
            }
        }
        Ok(self.state.status)
    }

    fn halt(&mut self, _timeout: Duration) -> Result<CoreInformation, Error> {
        self.write_debug_control(0x05)?;
        let pc = u64::from(self.interface.read_pc()?);
        self.state.status = CoreStatus::Halted(HaltReason::Request);
        Ok(CoreInformation { pc })
    }

    fn run(&mut self) -> Result<(), Error> {
        self.write_breakpoint_payload()?;
        // Vendor tools use breakpoint-go mode when armed user breakpoints are active.
        self.write_debug_control(if self.has_active_breakpoints() {
            0x84
        } else {
            0x08
        })?;
        self.state.status = CoreStatus::Running;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), Error> {
        self.write_breakpoint_payload()?;
        self.write_debug_control(0x88)?;
        self.state.status = CoreStatus::Running;
        Ok(())
    }

    fn reset_and_halt(&mut self, timeout: Duration) -> Result<CoreInformation, Error> {
        self.reset()?;
        self.halt(timeout)
    }

    fn step(&mut self) -> Result<CoreInformation, Error> {
        self.write_breakpoint_payload()?;
        self.write_debug_control(0x06)?;
        self.interface
            .write_memory(u64::from(REG_DEBUG_STEP), &[0x80])?;
        let pc = u64::from(self.interface.read_pc()?);
        self.state.status = CoreStatus::Halted(HaltReason::Step);
        Ok(CoreInformation { pc })
    }

    fn read_core_reg(&mut self, address: RegisterId) -> Result<RegisterValue, Error> {
        let snapshot = self.read_register_snapshot()?;
        let index = usize::from(address.0);
        let value = snapshot
            .get(index)
            .copied()
            .ok_or_else(|| Error::Register(format!("Unknown TC32 register {}", address.0)))?;
        Ok(RegisterValue::U32(value))
    }

    fn write_core_reg(&mut self, address: RegisterId, _value: RegisterValue) -> Result<(), Error> {
        Err(Error::NotImplemented(match address.0 {
            15 => "TC32 write program counter",
            _ => "TC32 write core register",
        }))
    }

    fn available_breakpoint_units(&mut self) -> Result<u32, Error> {
        Ok(BREAKPOINT_LOGICAL_LIMIT as u32)
    }

    fn hw_breakpoints(&mut self) -> Result<Vec<Option<u64>>, Error> {
        Ok(self.state.hardware_breakpoints.to_vec())
    }

    fn enable_breakpoints(&mut self, state: bool) -> Result<(), Error> {
        self.state.breakpoints_enabled = state;
        self.write_breakpoint_payload()
    }

    fn set_hw_breakpoint(&mut self, unit_index: usize, addr: u64) -> Result<(), Error> {
        let slot = self
            .state
            .hardware_breakpoints
            .get_mut(unit_index)
            .ok_or_else(|| {
                Error::Other(format!("TC32 breakpoint unit {unit_index} out of range"))
            })?;
        *slot = Some(addr);
        self.write_breakpoint_payload()
    }

    fn clear_hw_breakpoint(&mut self, unit_index: usize) -> Result<(), Error> {
        let slot = self
            .state
            .hardware_breakpoints
            .get_mut(unit_index)
            .ok_or_else(|| {
                Error::Other(format!("TC32 breakpoint unit {unit_index} out of range"))
            })?;
        *slot = None;
        self.write_breakpoint_payload()
    }

    fn registers(&self) -> &'static CoreRegisters {
        &registers::TC32_CORE_REGISTERS
    }

    fn program_counter(&self) -> &'static CoreRegister {
        &registers::PC
    }

    fn frame_pointer(&self) -> &'static CoreRegister {
        &registers::R7
    }

    fn stack_pointer(&self) -> &'static CoreRegister {
        &registers::SP
    }

    fn return_address(&self) -> &'static CoreRegister {
        &registers::LR
    }

    fn hw_breakpoints_enabled(&self) -> bool {
        self.state.breakpoints_enabled
    }

    fn architecture(&self) -> probe_rs_target::Architecture {
        probe_rs_target::Architecture::Tc32
    }

    fn core_type(&self) -> CoreType {
        CoreType::Tc32
    }

    fn instruction_set(&mut self) -> Result<InstructionSet, Error> {
        Ok(InstructionSet::Tc32)
    }

    fn fpu_support(&mut self) -> Result<bool, Error> {
        Ok(false)
    }

    fn floating_point_register_count(&mut self) -> Result<usize, Error> {
        Ok(0)
    }

    fn reset_catch_set(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("TC32 reset catch"))
    }

    fn reset_catch_clear(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("TC32 reset catch"))
    }

    fn debug_core_stop(&mut self) -> Result<(), Error> {
        Ok(())
    }

    fn is_64_bit(&self) -> bool {
        false
    }

    fn spill_registers(&mut self) -> Result<(), Error> {
        Ok(())
    }
}
