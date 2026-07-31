# AtomicMatrix

### *To lock or not to lock. That is the question* - Shakespear... probably

AtomicMatrix is a lock-free shared memory arena based on Linux SHM scope. It enables separate processes to acess and manage the same memory state concurrently, while avoiding kernel involvement.

# What it is

AtomicMatrix works as a general purpose memory allocator that can be mapped independently by multiple threads on the same scope, or separate processes. The arena is backed by a linux file stored at `/dev/shm` dynamically generated at bootstrap, although callers can name it whatever their use case requires. At this point, all allocation and deallocation procedures between processes are handled safely through atomic operators, data manipulation on allocated cells can also take advantage of this atomic safety by using the header state machine to synchronize access through the `MatrixHandler` object.

# What makes it different

The matrix provides all the synchronization necessary for safely allocating and deallocating without getting in the way of other processes, while giving enough access to the caller to make it adaptable to a wide variety of use cases without imposing its own opinion on how your program should work. The phylosophy is that the internal operations should be fast, safe, and unopinionated enough that manage states between processes is not a problem the caller has to deal with. With this ideas in mind, we provide the following features:

- Blocks have no arbitrary sizes. The caller can allocate a cell to whatever size its data requires.
- Data blocks can be accessed simultaneously regardless of synchronization. Callers can synchronize threads and processes in any way their use case requires.
- The arena is completelly type agnostic. Whatever you throw in will be converted to bytes and written within the segment.
- MatrixHandler abstracts a lot of the manual work required to both write data, and validate written types and block validity without getting in the way

# How it works

## Memory layout

```text
[ Init Guard (16b) ] [ AtomicMatrix Struct (bitmaps, metadata, etc) ] [ padding ] [ Memory Arena ]
```

The arena starts with a 16b atomic init guard. Participants that attach to an arena will either create the file and initialize all the structures required, or loop until the first arriving participant finishes bootstraping everything.

The AtomicMatrix struct is written right after the init guard and works as the metadata all participants will use to get or insert free offsets.

All subsequent segments are the free memory arena that participants will use to allocate data.

## O(1) Allocation

The allocation algorithm uses a two-level segregated fit (TLSF) inspired atomic bitmap. The first level indexes power-of-two size classes, and the second level subdivides each class into 8 linear buckets until the next size class. If more than one block of the same size bucket is freed simultaneously, it adds the head of the bucket to the freed chain in the block header and pushes the current block as the new head of the bucket. This ensures that allocations are O(1) time complexity and fully thread safe with CAS (Compare and Swap) operations.

## Memory Healing

AtomicMatrix uses a custom coalescing engine we call **Kinetic Coalescing**. Each `ack` call triggers a *ripple* that travels leftwards towards the sector beginning, merging with all physically adjacent neighbours that are marked in a free state. Kinetic Coalescing follow 4 properties to make it safe under concurrent access:

- **Monotonicity**: Merging only travels backwards to the sector origin, eliminating circular dependencies, deadlocks, and merge-stop synchronization.
- **Permissive Concurrency**: If a process encounters contention on merging a neighbour, it stops the coalescing, merge whatever it got up to that point and complete the operation. If contention happens on allocation, it simply moves to the next block in the available chain or get a new one from the bitmap instead of locking or waiting.
- **Pressure-driven healing**: `ack()` marks a block free and immediately starts the coalescing procedure, instead of relying on background cleanup threads or shared queues.
- **Local failures only**: If any failures arises from the operations (corrupted data, arena out of memory, etc), processes fail locally instead of blowing everything up (borrowed directly from the Actor Model).

## Process independent addressing

All pointers are stored as `u32` offsets relative to the base of the SHM segment, and wrapped in a `RelativePtr` struct. Each process can use this to resolve coordinates against its own mapping.

## Block lifecycle

```text
STATE_FREE -> (block is allocated) -> STATE_ALLOCATED -> (ack is called) -> STATE_ACKED -> STATE_COALESCING -> STATE_FREE
```

Blocks relies on an atomic state machine stored in the header. Each allocation/deallocation op moves the state machine forward, granting that blocks live exactly the way the caller intended to.

### Correctness

The state machine grants that the data in the block cannot be overwritten or cleaned up by processes that do not have ownership over that specific block. If a neighbour is freed, it will not touch the current block, unless it is in STATE_FREE or STATE_ACKED. Coalescing also ignores the data section of blocks, jumping directly into the previous block header instead, using the physical neighbour chain present in each header.

It also grants that no coalescing procedures overlap each other, since it only considers the two aformentioned states for merging.

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

AtomicMatrix is in its early stages of development, and its not available in crates.io yet. Using it requires getting it directly from github for now

```toml
[dependencies]
atomic-matrix = { git = "https://github.com/CodenamePhantom/AtomicMatrix" }
```

```rust
use atomic_matrix::prelude::*;

// Bootstrap a 50MB matrix in /dev/shm

let handler = AtomicMatrix::bootstrap(
    uid_lite::generate_uuid(), // internal uid generator because i hate have to import UUID to every other project.
    memory_scale::custom::mb::<50>(), // memory layout creator because i love semantic sugar.
).unwrap();

// Allocate a u64 block

let mut block = handler.allocate::<u64>().unwrap();

// Write the message. This will dump the bytes regardless of the block state.

unsafe { handler.write(&mut block, 2000).unwrap() };

// You can also safe write using atomic states (States from 0 to 49 are reserved and will fail if tried)

handler.write_if(&mut block, 2000, MyCustomState).unwrap();
// or
handler.write_transition(&mut block, 2000, TargetState, MyCustomState, Ordering::Whatever).unwrap();

// Read it back

let data = handler.read_copy(&block).unwrap();
println!("{data}");

// You can also read type guarded references directly

let rt_ref = handler.read(&block).unwrap() // reference to T
let rt_mut_ref = handler.read_mut(&block).unwrap() // mutable reference to T

// references can be deref

match unwind!({ *rt_mut_ref = heavy_math_equation(*rt_mut_ref) }) { // unwind! casts panics into a HandlerErrors
    Ok(_) => {
        println!("{}", *rt_ref);
    },
    Err(_) => {
        println!("Block deallocated");
    }
}

// Free it

handler.free(block);

// Kill the arena

unsafe { handler.die().unwrap() }
```

Multiple processes can map the same segment by passing the same UUID to `bootstrap`. The init guard ensures only one process performs the initial formatting regardless of how many processes call `bootstrap` simultaneously.

## Safety

AtomicMatrix is `unsafe` at the boundary layer. The MatrixHandler API contains that unsafety behind `Block<T>` abstractions that enforce offset validity within segment bounds with checked queries, caller state validations and data transitions. Although, unsafe layers are also provided for strict caller synchronization when needed.

We also provide a direct escape hatch into the raw matrix struct for direct arena manipulation (use with caution).

## Disclaimer

This project is not in a state that i would consider safe for production usage. It is a very new crate and concept that would require many tests and iterations to be considered battle-hardened. I wont stop you from doing it (although i'll judge you... a lot), but i'll refrain from any issues acquired from using it.

Also, pardon my french in some of the code comments. I have a personality and i'm not afraid of using it lol.

## License

Apache-2.0
