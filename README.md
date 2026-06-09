# AtomicMatrix

AtomicMatrix is a lock-free shared-memory allocator for zero-copy IPC between processes on the same machine. No native mutexes and no kernel involvement on the hot path.

## What it is

AtomicMatrix is a general-purpose lock-free memory allocator that lives in `/dev/shm`. Multiple independent processes map the same segment and allocate, read, and free blocks through a unified API with no copying and no kernel involvement on the hot path.

It is designed as the memory fabric for low-latency IPC pipelines - the kind of infrastructure that sits underneath a database write buffer, a real-time event bus, or a shared state mesh across microservices.

## What makes it different

Most high-performance allocators make one of three sacrifices: they are fast but not general (fixed-size ring buffers), general but not fast (mutex-based), or fast and general but operationally complex (DPDK mempool, requiring specific NICs and a full DPDK stack).

AtomicMatrix is a general-purpose allocator with built in arbitrary sizes, O(1) allocation and free, running at wire speed on commodity hardware, deployable as a single Rust crate.

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
[ Init Guard (16b) ] [ AtomicMatrix struct ] [ Padding ] [ Allocation Sector ]
```

All metadata related to the matrix is stored at the very beginning to be globally accessible across modules, acting as a shared state manager coordinating actions between concurrent requests.

### Allocation: O(1)

Allocation uses a two-level segregated fit (TLSF) inspired bitmap. A first-level bitmap indexes power-of-two size classes. A second-level bitmap subdivides each class into 8 linear steps. Finding a suitable block is two bitmap operations and one CAS. No scanning, no sorting, no locks.

### Memory Healing

Freeing a block triggers a backward **Ripple**. A coalescing traversal that merges the freed block with its left physical neighbours as long as they are free or acknowledged. Three properties make this safe under concurrent access:

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

**Hardware**: Intel Core i7 7th generation, 16GB DDR4
**OS**: Linux Fedora 43, `/dev/shm` backed
**Kernel**: 6.19.11-200.fc43.x86_64 (64-bit CPU)
**Build**: `cargo test --release`

### Endurance (600 seconds, 8 threads, mixed workload)

```
Total operations: 9,181,958,716
Throughput: 15.30 Mop/s
Final free fragments: 140
Entropy percentage: 0.0000015247291381962252%
```

Workload: randomized allocation sizes (32–544 bytes), 70/30 alloc/free ratio, randomized free ordering to stress the coalescing engine. You can check by running:

```bash
cargo test --lib matrix::tests::test_long_term_fragmentation_healing -- --include_ignored --no-capture
```

### Stats (60s of random workload - Per Thread scaling)

| Threads | Total Ops | Throughput (Mop/s) | ns/op | Scaling vs 1T | Efficiency |
|---------|-----------|-------------------|-------|---------------|------------|
| 1 | 272,141,174 | 4.54 | 220.3 | 1.00x | 100% |
| 2 | 431,437,387 | 7.19 | 139.1 | 1.58x | 79% |
| 4 | 720,029,358 | 12.00 | 83.3 | 2.64x | 66% |
| 8 | 1,104,515,810 | 18.41 | 54.3 | 4.05x | 51% |
| 16 | 994,983,604 | 16.58 | 60.3 | 3.65x | 23% |




> **Attention**
> The long term healing test is a stress test that produces a lot of workload in the CPU. It is recommended to adapt the quantity of concurrent threads to the declared number of vThreads provided by your manufacturer.

---

## Usage


```toml
[dependencies]
atomic-matrix = "0.1"
```

```rust
use atomic_matrix::prelude::*;

// Bootstrap a 50MB matrix in /dev/shm
let handler = AtomicMatrix::bootstrap(
    uid_lite::generate_uuid(), // internal uid generator because i hate have to import UUID to every other project.
    memory_scale::custom::mb::<7>(), // memory layout creator because i love semantic sugar.
).unwrap();

// Allocate a 128-byte block
let mut block = handler.allocate::<[u8; 128]>().unwrap();

// Write the message
let msg = b"Hello World!";
let mut payload = [0u8; 128];
payload[..msg.len()].copy_from_slice(msg);

unsafe { handler.write(&mut block, payload) }

// Read it back
let data = unsafe { handler.read(&block) }
println!("{}", std::str::from_utf8(&data[..12]).unwrap());

// Free it
handler.free(block);
```

Multiple processes can map the same segment by passing the same UUID to `bootstrap`. The init guard ensures only one process performs the initial formatting regardless of how many processes call `bootstrap` simultaneously.

---

## Roadmap

### v0.6 (under development) — Internals and Extensive Library for the matrix.
A set of pre-baked data structures and frameworks that can be used with the matrix to execute high-level actions (Iteration, IPC Semantics, etc).

> The following have already been implemented or is activelly under development:

- AtomicRingbuffer.
- MemoryScaler.
- UIDLite.
- AtomicExtensions.
- Fencer.
- Looper

For more information on how these works, check the documentation section (currently under development).
Check our [public development tracker](https://spiky-salad-0e7.notion.site/AtomicMatrix-3786c5a79f1e809893addd364b70ecf9)!

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

## Contributing

The standard library is the immediate area where contributions are welcome, as well as FFI bindings for other languages. If you want to build on top of the core allocator or have ideas for the API/Library design, open an issue.

---

## License

Apache-2.0
