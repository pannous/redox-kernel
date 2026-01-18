//! VFS-level path caching for Redox
//!
//! This module provides kernel-level caching to reduce filesystem driver overhead:
//! 1. Negative dentry cache - remember paths that don't exist (ENOENT)
//! 2. Path hint cache - remember path → node_id mappings for fast lookup hints
//!
//! The cache significantly improves performance by avoiding repeated tree walks
//! in userspace filesystem drivers.

use alloc::{collections::BTreeMap, string::String, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};

use crate::{
    scheme::SchemeNamespace,
    sync::{CleanLockToken, RwLock, L1},
    time,
};

/// Time-to-live for negative cache entries (in nanoseconds)
/// Failed lookups are cached for 5 seconds
const NEGATIVE_CACHE_TTL_NS: u64 = 5_000_000_000;

/// Time-to-live for positive cache entries (in nanoseconds)
/// Successful lookups are cached for 30 seconds
const POSITIVE_CACHE_TTL_NS: u64 = 30_000_000_000;

/// Maximum number of cache entries (to bound memory usage)
const MAX_CACHE_ENTRIES: usize = 4096;

/// A cached path lookup result
#[derive(Clone, Debug)]
pub enum CachedLookup {
    /// Path does not exist - return ENOENT immediately
    NotFound {
        /// When this entry expires (monotonic nanoseconds)
        expires_at: u64,
    },
    /// Path exists - hint for filesystem driver
    Found {
        /// Scheme-specific node identifier (e.g., inode number in RedoxFS)
        node_id: u64,
        /// When this entry expires (monotonic nanoseconds)
        expires_at: u64,
    },
}

impl CachedLookup {
    fn is_expired(&self, now: u64) -> bool {
        match self {
            CachedLookup::NotFound { expires_at } => now >= *expires_at,
            CachedLookup::Found { expires_at, .. } => now >= *expires_at,
        }
    }
}

/// Cache key combining namespace, scheme, and path
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct CacheKey {
    namespace: usize,
    scheme: String,
    path: String,
}

/// The global VFS cache
pub struct VfsCache {
    /// Cache entries indexed by (namespace, scheme, path)
    entries: BTreeMap<CacheKey, CachedLookup>,
    /// Number of cache hits (for statistics)
    hits: AtomicU64,
    /// Number of cache misses (for statistics)
    misses: AtomicU64,
    /// Number of negative cache hits (for statistics)
    negative_hits: AtomicU64,
}

impl VfsCache {
    pub const fn new() -> Self {
        Self {
            entries: BTreeMap::new(),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
            negative_hits: AtomicU64::new(0),
        }
    }

    /// Look up a path in the cache
    pub fn lookup(&mut self, namespace: SchemeNamespace, scheme: &str, path: &str) -> Option<CachedLookup> {
        let key = CacheKey {
            namespace: namespace.into(),
            scheme: scheme.into(),
            path: path.into(),
        };

        let now = current_time_ns();

        if let Some(entry) = self.entries.get(&key) {
            if entry.is_expired(now) {
                // Entry expired, remove it
                self.entries.remove(&key);
                self.misses.fetch_add(1, Ordering::Relaxed);
                None
            } else {
                // Valid cache hit
                self.hits.fetch_add(1, Ordering::Relaxed);
                if matches!(entry, CachedLookup::NotFound { .. }) {
                    self.negative_hits.fetch_add(1, Ordering::Relaxed);
                }
                Some(entry.clone())
            }
        } else {
            self.misses.fetch_add(1, Ordering::Relaxed);
            None
        }
    }

    /// Insert a negative (ENOENT) cache entry
    pub fn insert_negative(&mut self, namespace: SchemeNamespace, scheme: &str, path: &str) {
        self.maybe_evict();

        let key = CacheKey {
            namespace: namespace.into(),
            scheme: scheme.into(),
            path: path.into(),
        };

        let now = current_time_ns();
        self.entries.insert(key, CachedLookup::NotFound {
            expires_at: now + NEGATIVE_CACHE_TTL_NS,
        });
    }

    /// Insert a positive cache entry with node_id hint
    pub fn insert_found(&mut self, namespace: SchemeNamespace, scheme: &str, path: &str, node_id: u64) {
        self.maybe_evict();

        let key = CacheKey {
            namespace: namespace.into(),
            scheme: scheme.into(),
            path: path.into(),
        };

        let now = current_time_ns();
        self.entries.insert(key, CachedLookup::Found {
            node_id,
            expires_at: now + POSITIVE_CACHE_TTL_NS,
        });
    }

    /// Invalidate cache entries for a path (on file creation, deletion, rename)
    pub fn invalidate(&mut self, namespace: SchemeNamespace, scheme: &str, path: &str) {
        let key = CacheKey {
            namespace: namespace.into(),
            scheme: scheme.into(),
            path: path.into(),
        };
        self.entries.remove(&key);
    }

    /// Invalidate all cache entries for a scheme (e.g., when filesystem unmounts)
    pub fn invalidate_scheme(&mut self, namespace: SchemeNamespace, scheme: &str) {
        let ns: usize = namespace.into();
        self.entries.retain(|k, _| !(k.namespace == ns && k.scheme == scheme));
    }

    /// Invalidate cache entries matching a path prefix (for directory operations)
    pub fn invalidate_prefix(&mut self, namespace: SchemeNamespace, scheme: &str, prefix: &str) {
        let ns: usize = namespace.into();
        let scheme_str: String = scheme.into();
        self.entries.retain(|k, _| {
            !(k.namespace == ns && k.scheme == scheme_str && k.path.starts_with(prefix))
        });
    }

    /// Evict old entries if cache is too large
    fn maybe_evict(&mut self) {
        if self.entries.len() >= MAX_CACHE_ENTRIES {
            // Remove expired entries first
            let now = current_time_ns();
            self.entries.retain(|_, v| !v.is_expired(now));

            // If still too large, remove oldest entries (first 25%)
            if self.entries.len() >= MAX_CACHE_ENTRIES {
                let to_remove = self.entries.len() / 4;
                let keys: Vec<_> = self.entries.keys().take(to_remove).cloned().collect();
                for key in keys {
                    self.entries.remove(&key);
                }
            }
        }
    }

    /// Get cache statistics
    pub fn stats(&self) -> (u64, u64, u64, usize) {
        (
            self.hits.load(Ordering::Relaxed),
            self.misses.load(Ordering::Relaxed),
            self.negative_hits.load(Ordering::Relaxed),
            self.entries.len(),
        )
    }
}

/// Get current monotonic time in nanoseconds
fn current_time_ns() -> u64 {
    // Use kernel's time module
    let timespec = time::monotonic();
    timespec.tv_sec as u64 * 1_000_000_000 + timespec.tv_nsec as u64
}

/// Global VFS cache instance
static VFS_CACHE: RwLock<L1, VfsCache> = RwLock::new(VfsCache::new());

/// Look up a path in the global VFS cache
pub fn cache_lookup(
    namespace: SchemeNamespace,
    scheme: &str,
    path: &str,
    token: &mut CleanLockToken,
) -> Option<CachedLookup> {
    VFS_CACHE.write(token.token()).lookup(namespace, scheme, path)
}

/// Insert a negative (ENOENT) result into the cache
pub fn cache_insert_negative(
    namespace: SchemeNamespace,
    scheme: &str,
    path: &str,
    token: &mut CleanLockToken,
) {
    VFS_CACHE.write(token.token()).insert_negative(namespace, scheme, path);
}

/// Insert a positive result with node_id hint into the cache
pub fn cache_insert_found(
    namespace: SchemeNamespace,
    scheme: &str,
    path: &str,
    node_id: u64,
    token: &mut CleanLockToken,
) {
    VFS_CACHE.write(token.token()).insert_found(namespace, scheme, path, node_id);
}

/// Invalidate a specific path in the cache
pub fn cache_invalidate(
    namespace: SchemeNamespace,
    scheme: &str,
    path: &str,
    token: &mut CleanLockToken,
) {
    VFS_CACHE.write(token.token()).invalidate(namespace, scheme, path);
}

/// Invalidate all entries for a scheme
pub fn cache_invalidate_scheme(
    namespace: SchemeNamespace,
    scheme: &str,
    token: &mut CleanLockToken,
) {
    VFS_CACHE.write(token.token()).invalidate_scheme(namespace, scheme);
}

/// Invalidate entries matching a path prefix
pub fn cache_invalidate_prefix(
    namespace: SchemeNamespace,
    scheme: &str,
    prefix: &str,
    token: &mut CleanLockToken,
) {
    VFS_CACHE.write(token.token()).invalidate_prefix(namespace, scheme, prefix);
}

/// Get cache statistics: (hits, misses, negative_hits, entries)
pub fn cache_stats(token: &mut CleanLockToken) -> (u64, u64, u64, usize) {
    VFS_CACHE.read(token.token()).stats()
}
