//! Null scheme for measuring in-kernel syscall overhead.
//!
//! This scheme does minimal work - useful for benchmarking pure syscall/IPC overhead
//! without filesystem or driver overhead.
//!
//! Usage from userspace:
//!   open("null:") - returns a handle immediately
//!   read(fd, buf, n) - returns 0 (EOF) immediately
//!   write(fd, buf, n) - returns n (success) immediately
//!   close(fd) - returns success immediately

use core::sync::atomic::{AtomicUsize, Ordering};

use crate::{
    scheme::*,
    sync::{CleanLockToken, RwLock, L1},
    syscall::{
        data::Stat,
        flag::MODE_FILE,
        usercopy::{UserSliceRo, UserSliceWo},
    },
};

static NEXT_ID: AtomicUsize = AtomicUsize::new(1);

static HANDLES: RwLock<L1, HashMap<usize, ()>> =
    RwLock::new(HashMap::with_hasher(DefaultHashBuilder::new()));

pub struct NullScheme;

impl KernelScheme for NullScheme {
    fn kopen(
        &self,
        _path: &str,
        _flags: usize,
        _ctx: CallerCtx,
        token: &mut CleanLockToken,
    ) -> Result<OpenResult> {
        // Minimal work: allocate an ID and insert into handles map
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        HANDLES.write(token.token()).insert(id, ());
        Ok(OpenResult::SchemeLocal(id, InternalFlags::empty()))
    }

    fn kread(
        &self,
        id: usize,
        _buf: UserSliceWo,
        _flags: u32,
        _stored_flags: u32,
        token: &mut CleanLockToken,
    ) -> Result<usize> {
        // Verify handle exists
        let handles = HANDLES.read(token.token());
        if !handles.contains_key(&id) {
            return Err(Error::new(EBADF));
        }
        // Return 0 = EOF, no data copied
        Ok(0)
    }

    fn kwrite(
        &self,
        id: usize,
        buf: UserSliceRo,
        _flags: u32,
        _stored_flags: u32,
        token: &mut CleanLockToken,
    ) -> Result<usize> {
        // Verify handle exists
        let handles = HANDLES.read(token.token());
        if !handles.contains_key(&id) {
            return Err(Error::new(EBADF));
        }
        // Return full length = all bytes "written"
        Ok(buf.len())
    }

    fn kfpath(&self, id: usize, buf: UserSliceWo, token: &mut CleanLockToken) -> Result<usize> {
        let handles = HANDLES.read(token.token());
        if !handles.contains_key(&id) {
            return Err(Error::new(EBADF));
        }
        const PATH: &[u8] = b"null:";
        let len = core::cmp::min(buf.len(), PATH.len());
        buf.limit(len)
            .expect("must succeed")
            .copy_from_slice(&PATH[..len])?;
        Ok(len)
    }

    fn kfstat(&self, id: usize, buf: UserSliceWo, token: &mut CleanLockToken) -> Result<()> {
        let handles = HANDLES.read(token.token());
        if !handles.contains_key(&id) {
            return Err(Error::new(EBADF));
        }
        let stat = Stat {
            st_mode: MODE_FILE | 0o666,
            st_size: 0,
            st_blksize: 4096,
            st_blocks: 0,
            ..Default::default()
        };
        buf.copy_exactly(&stat)?;
        Ok(())
    }

    fn fsync(&self, id: usize, token: &mut CleanLockToken) -> Result<()> {
        let handles = HANDLES.read(token.token());
        if !handles.contains_key(&id) {
            return Err(Error::new(EBADF));
        }
        Ok(())
    }

    fn close(&self, id: usize, token: &mut CleanLockToken) -> Result<()> {
        let mut handles = HANDLES.write(token.token());
        handles.remove(&id).ok_or(Error::new(EBADF))?;
        Ok(())
    }
}
