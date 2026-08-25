// SPDX-License-Identifier: GPL-2.0

//! Controller-wide UFS command slot management.

use kernel::prelude::*;

pub(crate) const TASK_TAG_COUNT: usize = 1usize << u8::BITS;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct TaskTag(u8);

impl TaskTag {
    pub(crate) fn new(tag: u32) -> Result<Self> {
        Ok(Self(u8::try_from(tag).map_err(|_| EINVAL)?))
    }

    pub(crate) fn from_index(tag: usize) -> Result<Self> {
        Ok(Self(u8::try_from(tag).map_err(|_| EINVAL)?))
    }

    pub(crate) fn value(self) -> u8 {
        self.0
    }

    pub(crate) fn index(self) -> usize {
        usize::from(self.0)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CommandOwner {
    pub(crate) queue_id: u32,
    pub(crate) blk_tag: u32,
}

#[derive(Clone, Copy)]
enum CommandSlotState {
    Free,
    Reserved,
    Bound(CommandOwner),
}

pub(crate) struct CommandPool {
    slots: [CommandSlotState; TASK_TAG_COUNT],
    tag_count: usize,
    max_active: usize,
    active: usize,
}

impl CommandPool {
    pub(crate) fn new(tag_count: usize, max_active: usize) -> Result<Self> {
        if tag_count == 0
            || tag_count > TASK_TAG_COUNT
            || max_active == 0
            || max_active > tag_count
        {
            return Err(EINVAL);
        }

        Ok(Self {
            slots: [CommandSlotState::Free; TASK_TAG_COUNT],
            tag_count,
            max_active,
            active: 0,
        })
    }

    pub(crate) fn reserve(&mut self) -> Option<TaskTag> {
        if self.active == self.max_active {
            return None;
        }

        for (tag, slot) in self.slots[..self.tag_count].iter_mut().enumerate() {
            if matches!(slot, CommandSlotState::Free) {
                *slot = CommandSlotState::Reserved;
                self.active += 1;
                return u8::try_from(tag).ok().map(TaskTag);
            }
        }
        None
    }

    pub(crate) fn bind(&mut self, task_tag: TaskTag, owner: CommandOwner) -> Result<()> {
        let slot = self
            .slots
            .get_mut(task_tag.index())
            .ok_or(EINVAL)?;
        if !matches!(slot, CommandSlotState::Reserved) {
            return Err(EIO);
        }
        *slot = CommandSlotState::Bound(owner);
        Ok(())
    }

    pub(crate) fn owner(&self, task_tag: TaskTag) -> Option<CommandOwner> {
        match self.slots.get(task_tag.index())? {
            CommandSlotState::Bound(owner) => Some(*owner),
            _ => None,
        }
    }

    pub(crate) fn recovery_owner(&self, task_tag: TaskTag) -> Result<Option<CommandOwner>> {
        match self.slots.get(task_tag.index()).ok_or(EINVAL)? {
            CommandSlotState::Free => Ok(None),
            CommandSlotState::Reserved => Err(EBUSY),
            CommandSlotState::Bound(owner) => Ok(Some(*owner)),
        }
    }

    pub(crate) fn release(&mut self, task_tag: TaskTag) -> Result<()> {
        let slot = self
            .slots
            .get_mut(task_tag.index())
            .ok_or(EINVAL)?;
        if matches!(slot, CommandSlotState::Free) {
            return Err(EIO);
        }
        let active = self.active.checked_sub(1).ok_or(EIO)?;
        *slot = CommandSlotState::Free;
        self.active = active;
        Ok(())
    }

    pub(crate) fn active(&self) -> usize {
        self.active
    }
}
