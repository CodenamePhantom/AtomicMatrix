# AtomicMatrix

A lock-free, shared-memory-backed memory allocator for high-performance inter-process communication (IPC). Built in Rust on a TLSF-inspired two-level bitmap with a custom kinetic coalescing engine.

**14.77 Mop/s sustained throughput on a 2017 consumer i7. No kernel patches. No specialized hardware.**

> **v0.1 — Core allocator layer.** The low-level engine is complete and benchmarked. A high-level message API (`post`, `inquire`, `read`) and a policy trait layer are actively in development and will ship in v0.2.

---

## What it is

AtomicMatrix is a general-purpose lock-free memory allocator that lives in `/dev/shm`. Multiple independent processes map the same segment and allocate, read, and free blocks through a unified API with no copying and no kernel involvement on the hot path.

It is designed as the memory fabric for low-latency IPC pipelines - the kind of infrastructure that sits underneath a database write buffer, a real-time event bus, or a shared state mesh across microservices.

## What makes it different

Most high-performance allocators make one of three sacrifices: they are fast but not general (fixed-size ring buffers), general but not fast (mutex-based), or fast and general but operationally complex (DPDK mempool, requiring specific NICs and a full DPDK stack).

AtomicMatrix is a general-purpose allocator - arbitrary sizes, O(1) allocation and free - running at wire speed on commodity hardware, deployable as a single Rust crate with no external dependencies beyond the OS.

| | Lock-free | Arbitrary sizes | SHM-backed | Commodity hardware |
|---|---|---|---|---|
| **AtomicMatrix** | ✓ | ✓ | ✓ | ✓ |
| LMAX Disruptor | ✓ | ✗ (fixed) | ✗ | ✓ |
| Intel DPDK mempool | ✓ | ✗ (fixed) | ✗ | ✗ |
| Boost.Interprocess | ✗ | ✓ | ✓ | ✓ |

---

## How it works

### Memory layout

```
[ Init Guard (16b) ] [ AtomicMatrix struct ] [ Padding ] [ Sector 0 ] [ Sector 1 ] [ Sector 2 ]
```

The SHM segment is sectorized into three zones at initialization - small, medium, and large - based on configurable size percentages. Each sector is a self-contained fault domain. Fragmentation in one sector cannot propagate into another.

### Allocation: O(1)

Allocation uses a two-level segregated fit (TLSF) inspired bitmap. A first-level bitmap indexes power-of-two size classes. A second-level bitmap subdivides each class into 8 linear steps. Finding a suitable block is two bitmap operations and one CAS - no scanning, no sorting, no locks.

### The Propagation Principle of Atomic Coalescence

Freeing a block triggers a backward **Ripple** - a coalescing traversal that merges the freed block with its left physical neighbours as long as they are free or acknowledged. Three properties make this safe under concurrent access:

- **Monotonicity**: Ripples only propagate backward toward the sector origin, eliminating circular dependencies and deadlock.
- **Permissive concurrency**: If a thread encounters contention on a neighbour, it stops the ripple and moves on. A future operation will complete the merge.
- **Pressure-driven healing**: `ack()` marks a block free and immediately attempts coalescing. No background thread, no shared queue, no coordination overhead.

### Process-independent addressing

All pointers are stored as `u32` offsets relative to the base of the SHM segment. Each process resolves offsets against its own mapping. The same offset is valid in every process that has mapped the segment, regardless of where the OS placed the mapping in virtual address space.

### Block lifecycle

```
STATE_FREE → STATE_ALLOCATED → STATE_ACKED → STATE_COALESCING → STATE_FREE
```

- `allocate()` — transitions a block from FREE to ALLOCATED
- `ack()` — transitions ALLOCATED to ACKED and immediately triggers coalescing
- `coalesce()` — merges ACKED/FREE neighbours, transitions merged block to FREE

---

## Benchmarks

All benchmarks run on a single node with no NUMA tuning, no kernel patches, and no huge pages (default configuration).

**Hardware**: Intel Core i7 7th generation, DDR4
**OS**: Linux, `/dev/shm` backed
**Build**: `cargo test --release`

### Endurance (600 seconds, 8 threads, mixed workload)

```
Duration:        600 seconds
Threads:         8
Total ops:       8,860,017,740
Throughput:      14.77 Mop/s
Free fragments:  118 / 256 cells
Entropy:         0.000001%
```

Workload: randomized allocation sizes (32–544 bytes), 70/30 alloc/free ratio, randomized free ordering to stress the coalescing engine.

---

## Usage

> **Note**: v0.1 exposes the low-level allocator API directly. A higher-level message API is coming in v0.2. If you want to build your own abstractions on top of the core, this is the right entry point.

```toml
[dependencies]
atomic-matrix = "0.1"
```

```rust
use atomic_matrix::matrix::core::{AtomicMatrix, RelativePtr, BlockHeader};

// Bootstrap a 50MB matrix in /dev/shm
let handler = AtomicMatrix::bootstrap(
    Some(uuid::Uuid::new_v4()),
    50 * 1024 * 1024,
    (40, 30)  // 40% small sector, 30% medium sector, 30% large sector
).unwrap();

let base_ptr = handler.mmap.as_ptr();

// Allocate a 128-byte block
let ptr = handler.matrix.allocate(base_ptr, 128).unwrap();

// Free it
let header_ptr = RelativePtr::<BlockHeader>::new(ptr.offset() - 32);
handler.matrix.ack(&header_ptr, base_ptr);
```

Multiple processes can map the same segment by passing the same UUID to `bootstrap`. The init guard ensures only one process performs the initial formatting regardless of how many processes call `bootstrap` simultaneously.

---

## Features

```toml
[features]
default = []
server = ["libc"]   # enables huge page mapping for server deployments
```

The `server` feature enables `MADV_HUGEPAGE` on the SHM mapping. Recommended for bare-metal server deployments with large matrices. Not recommended for edge or resource-constrained environments.

```bash
cargo build --features server
```

---

## Roadmap

### v0.2 — High-level message API
A typed message layer built on top of the core allocator:
- `post()` — write a message into the matrix
- `inquire()` — query for available messages
- `read()` — consume a message with zero-copy semantics

### v0.3 — Policy trait layer
A trait-based policy system for typed message enums, allowing callers to define routing, priority, and retention behavior as composable traits attached to message types.

---

## Safety

AtomicMatrix is `unsafe` at the boundary layer — pointer arithmetic over a raw SHM segment is inherently unsafe. The public API contains that unsafety behind `RelativePtr<T>` abstractions that enforce offset validity within segment bounds.

All state transitions use atomic operations with explicit memory ordering. No mutexes. No `UnsafeCell` wrappers. All shared state is `AtomicU32`.

The allocator has been tested under:
- 8.8 billion operations over 600 seconds across 8 threads
- Concurrent allocation and deallocation with randomized sizes
- Randomized free ordering to stress the coalescing engine
- Race condition detection via unique offset tracking across threads

---

## Theoretical foundation

The design and its formal properties are described in the accompanying paper:

> *Causality as a Physical Limit: Arbitration, Uncertainty, and the Deterministic Distributed Memory Fabric*
> [ArXiv — coming soon]

---

## Contributing

The high-level API and policy layer are the immediate areas where contributions are welcome. If you want to build on top of the core allocator or have ideas for the message API design, open an issue.

---

## License

MIT OR Apache-2.0