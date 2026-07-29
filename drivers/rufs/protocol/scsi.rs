// SPDX-License-Identifier: GPL-2.0

//! SCSI command and completion protocol definitions.

pub(crate) const UFS_SENSE_SIZE: usize = 18;

#[derive(Clone, Copy, Debug)]
pub(crate) enum UfsScsiCompletion {
    Good,
    CheckCondition,
    Busy,
    ReservationConflict,
    TaskSetFull,
    TaskAborted,
    Requeue,
    Error,
}

#[derive(Clone, Copy)]
pub(crate) struct UfsScsiResult {
    pub(crate) completion: UfsScsiCompletion,
    pub(crate) ocs: u8,
    pub(crate) transaction: u8,
    pub(crate) response: u8,
    pub(crate) status: u8,
    pub(crate) residual_transfer_count: u32,
    pub(crate) sense_data_len: usize,
    pub(crate) sense_data: [u8; UFS_SENSE_SIZE],
}

impl UfsScsiResult {
    pub(crate) fn error(ocs: u8) -> Self {
        Self {
            completion: UfsScsiCompletion::Error,
            ocs,
            transaction: 0,
            response: 0,
            status: 0,
            residual_transfer_count: 0,
            sense_data_len: 0,
            sense_data: [0; UFS_SENSE_SIZE],
        }
    }

    pub(crate) fn requeue(ocs: u8) -> Self {
        Self {
            completion: UfsScsiCompletion::Requeue,
            ..Self::error(ocs)
        }
    }
}

const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const UNMAP: u8 = 0x42;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;

#[derive(PartialEq, Copy, Clone, Debug)]
pub(crate) enum UfsScsiDataDirection {
    None,
    Read,
    Write,
}

pub(crate) struct ScsiSense {
    pub(crate) response_code: u8,
    pub(crate) sense_key: u8,
    pub(crate) asc: u8,
    pub(crate) ascq: u8,
    pub(crate) additional_len: u8,
}

impl ScsiSense {
    pub(crate) fn is_unit_attention(&self) -> bool {
        self.sense_key == SCSI_SENSE_UNIT_ATTENTION
    }

    pub(crate) fn is_power_on_reset(&self) -> bool {
        self.is_unit_attention()
            && self.asc == SCSI_ASC_POWER_ON_RESET
            && self.ascq == SCSI_ASCQ_POWER_ON_RESET
    }
}

const SCSI_SENSE_UNIT_ATTENTION: u8 = 0x6;
const SCSI_ASC_POWER_ON_RESET: u8 = 0x29;
const SCSI_ASCQ_POWER_ON_RESET: u8 = 0x00;

pub(crate) fn parse_scsi_sense(data: &[u8], len: usize) -> Option<ScsiSense> {
    if len == 0 {
        return None;
    }

    let response_code = data[0] & 0x7f;
    match response_code {
        // Fixed format sense data.
        0x70 | 0x71 if len >= 14 => Some(ScsiSense {
            response_code,
            sense_key: data[2] & 0x0f,
            asc: data[12],
            ascq: data[13],
            additional_len: data[7],
        }),
        // Descriptor format sense data.
        0x72 | 0x73 if len >= 4 => Some(ScsiSense {
            response_code,
            sense_key: data[1] & 0x0f,
            asc: data[2],
            ascq: data[3],
            additional_len: 0,
        }),
        _ => None,
    }
}

pub(crate) fn sense_key_name(key: u8) -> &'static str {
    match key {
        0x0 => "NO_SENSE",
        0x1 => "RECOVERED_ERROR",
        0x2 => "NOT_READY",
        0x3 => "MEDIUM_ERROR",
        0x4 => "HARDWARE_ERROR",
        0x5 => "ILLEGAL_REQUEST",
        0x6 => "UNIT_ATTENTION",
        0x7 => "DATA_PROTECT",
        0x8 => "BLANK_CHECK",
        0x9 => "VENDOR_SPECIFIC",
        0xb => "ABORTED_COMMAND",
        0xd => "VOLUME_OVERFLOW",
        0xe => "MISCOMPARE",
        _ => "UNKNOWN",
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsSCSICmd {
    lun: u8,
    direction: UfsScsiDataDirection,
    data_len: u32,
    cdb: [u8; 16],
    unmap_lba: u64,
    unmap_blocks: u32,
}

impl UfsSCSICmd {
    pub(crate) fn read_write(
        lun: u8,
        write: bool,
        lba: u64,
        blocks: u32,
        data_len: u32,
        fua: bool,
    ) -> Self {
        let mut cdb = [0u8; 16];
        let direction = if write {
            UfsScsiDataDirection::Write
        } else {
            UfsScsiDataDirection::Read
        };
        let flags = if fua { 0x8 } else { 0 };

        match (u32::try_from(lba), u16::try_from(blocks)) {
            (Ok(lba), Ok(blocks)) => {
                cdb[0] = if write { WRITE_10 } else { READ_10 };
                cdb[1] = flags;
                cdb[2..6].copy_from_slice(&lba.to_be_bytes());
                cdb[7..9].copy_from_slice(&blocks.to_be_bytes());
            }
            _ => {
                cdb[0] = if write { WRITE_16 } else { READ_16 };
                cdb[1] = flags;
                cdb[2..10].copy_from_slice(&lba.to_be_bytes());
                cdb[10..14].copy_from_slice(&blocks.to_be_bytes());
            }
        }

        Self {
            lun,
            direction,
            data_len,
            cdb,
            unmap_lba: 0,
            unmap_blocks: 0,
        }
    }

    pub(crate) fn flush(lun: u8) -> Self {
        let mut cdb = [0u8; 16];
        cdb[0] = SYNCHRONIZE_CACHE;

        Self {
            lun,
            direction: UfsScsiDataDirection::None,
            data_len: 0,
            cdb,
            unmap_lba: 0,
            unmap_blocks: 0,
        }
    }

    pub(crate) fn unmap(lun: u8, lba: u64, blocks: u32) -> Self {
        let mut cdb = [0u8; 16];
        let data_len = 24u32;
        cdb[0] = UNMAP;
        cdb[7..9].copy_from_slice(&(data_len as u16).to_be_bytes());

        Self {
            lun,
            direction: UfsScsiDataDirection::Write,
            data_len,
            cdb,
            unmap_lba: lba,
            unmap_blocks: blocks,
        }
    }

    pub(crate) fn lun(&self) -> u8 {
        self.lun
    }

    pub(crate) fn direction(&self) -> UfsScsiDataDirection {
        self.direction
    }

    pub(crate) fn data_len(&self) -> u32 {
        self.data_len
    }

    pub(crate) fn cdb(&self) -> [u8; 16] {
        self.cdb
    }

    pub(crate) fn is_unmap(&self) -> bool {
        self.cdb[0] == UNMAP
    }

    pub(crate) fn unmap_lba(&self) -> u64 {
        self.unmap_lba
    }

    pub(crate) fn unmap_blocks(&self) -> u32 {
        self.unmap_blocks
    }
}
