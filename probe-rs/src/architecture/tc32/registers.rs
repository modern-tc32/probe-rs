//! TC32 register descriptions.

use std::sync::LazyLock;

use crate::{
    CoreRegister, CoreRegisters, RegisterId, RegisterRole,
    core::{RegisterDataType, UnwindRule},
};

macro_rules! tc32_reg {
    ($name:literal, $id:expr) => {
        CoreRegister {
            roles: &[RegisterRole::Core($name)],
            id: RegisterId($id),
            data_type: RegisterDataType::UnsignedInteger(32),
            unwind_rule: UnwindRule::Clear,
        }
    };
}

pub(crate) const R0: CoreRegister = tc32_reg!("r0", 0);
pub(crate) const R1: CoreRegister = tc32_reg!("r1", 1);
pub(crate) const R2: CoreRegister = tc32_reg!("r2", 2);
pub(crate) const R3: CoreRegister = tc32_reg!("r3", 3);
pub(crate) const R4: CoreRegister = tc32_reg!("r4", 4);
pub(crate) const R5: CoreRegister = tc32_reg!("r5", 5);
pub(crate) const R6: CoreRegister = tc32_reg!("r6", 6);
pub(crate) const R7: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("r7"), RegisterRole::FramePointer],
    id: RegisterId(7),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};
pub(crate) const R8: CoreRegister = tc32_reg!("r8", 8);
pub(crate) const R9: CoreRegister = tc32_reg!("r9", 9);
pub(crate) const R10: CoreRegister = tc32_reg!("r10", 10);
pub(crate) const R11: CoreRegister = tc32_reg!("r11", 11);
pub(crate) const R12: CoreRegister = tc32_reg!("r12", 12);
pub(crate) const SP: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("sp"), RegisterRole::StackPointer],
    id: RegisterId(13),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};
pub(crate) const LR: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("lr"), RegisterRole::ReturnAddress],
    id: RegisterId(14),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};
pub(crate) const PC: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("pc"), RegisterRole::ProgramCounter],
    id: RegisterId(15),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};
pub(crate) const PSR: CoreRegister = CoreRegister {
    roles: &[RegisterRole::Core("psr"), RegisterRole::ProcessorStatus],
    id: RegisterId(16),
    data_type: RegisterDataType::UnsignedInteger(32),
    unwind_rule: UnwindRule::Clear,
};

static TC32_REGISTERS_SET: &[CoreRegister] = &[
    R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, SP, LR, PC, PSR,
];

pub(crate) static TC32_CORE_REGISTERS: LazyLock<CoreRegisters> =
    LazyLock::new(|| CoreRegisters::new(TC32_REGISTERS_SET.iter().collect()));
