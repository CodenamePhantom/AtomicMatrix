### Table of contents
- Core
	- [Atomic Matrix](Core/Matrix.md)
	- [Handlers](Core/Handlers.md)
	- [Cartographer](Core/Cartographer.md)
- Internals
	- [Atomic Ringbuffer](Internals/AtomicRingbuffer.md)
	- [Atomic Array](Internals/AtomicArray.md)
	- [Errors](Internals/Errors.md)
- Extensive Library
	- [Looper](Extensive%20Lib/Looper.md)
	- [Fencer](Extensive%20Lib/Fencer.md)
	- [Guardian](Extensive%20Lib/Guardian.md)

## 1 - Objective

AtomicMatrix is a lock free memory arena based on Linux SHM protocol. It enables completely different processes to access the same memory space concurrently, providing seamless IPC and shared state management.

## 2 - Scope

AtomicMatrix aims to cover memory management safety (allocating/de-allocating) on a process agnostic arena, while providing enough flexibility to ensure adaptability to a wide range of use cases with a good subset of tools. These include:

- Safe O(1) allocation/de-allocation on cross-process concurrency.
- Block state machine for concurrent data manipulation synchronization.
- Shared state data structures such as ring buffers, arrays, maps, etc.
- Framework extensions for advanced behaviors.

The code for this infrastructure is divided into the modules Core, Internals, and Extensive Lib.

## 3 - Overview

### Core

The core module hosts the underlying plumb wire of the AtomicMatrix crate. The logic is separated into three main objects:

- **AtomicMatrix**: Responsible for creating or attaching to a SHM file, and coordinating concurrent memory management. It also holds the logic for raw offset arithmetic through the `RelativePtr` struct.
- **MatrixHandler**: Provides a safer typed layer above the AtomicMatrix, while abstracting a lot of the manual work required for parsing bytes into and from common data, and validating boundaries, life times, and type safety. It also provides a full set of unsafe escape hatches for fine tuning and nasty edge cases.
- **Cartographer**: Integrates SHM deployment with a lot of system safety features such as mprotect, LSM middlewares (AppArmor and SELinux), and MemFd raw descriptors. Providing a granular security declaration layer.

### Internals

Internals hosts the collection of thread safe data structures developed to be used inside AtomicMatrix. For now, only a few data structures have been added:

- Atomic Ringbuffer
- Atomic Array

### Extensive Lib

Extensive Lib builds upon the low and middle layers of the AtomicMatrix to provide higher level abstractions of work that can be used without much heavy lifting from the caller side. These are:

- **Fencer**: Multi SHM arena management, with ACL (Access Control List), message routing, and throughput volume balancing.
- **Guardian**: Byte level security features for critical data. Message encryption, size padding, key rotations, and all the sorts.
- **Looper**: Iter tools over matrix blocks... Yes, even under concurrency.
- **AtoMeter**: Memory management automation tools for inter process health check, memory balancing, and data cleanup.

### Basic Usage

AtomicMatrix is in its early stages of development, and its not available in `crates.io` yet. Using it requires getting it directly from GitHub for now

```toml
[dependencies]
atomic-matrix = { git = "https://github.com/CodenamePhantom/AtomicMatrix" }
```

By then, the caller can import the prelude module to load most of the important stuff.

```rust
use atomic_matrix::prelude::*;

// Bootstrap a 50MB matrix in /dev/shm

let handler = AtomicMatrix::bootstrap(
    uid_lite::generate_uuid(), // internal uid generator because i hate have to import UUID to every other project.
    memory_scale::custom::mb::<50>(), // memory layout creator because i love semantic sugar.
).unwrap();

// Allocate a u64 block
// Blocks always starts at STATE_ALLOCATED.

let mut block = handler.allocate::<u64>().unwrap();

// Write the message. This will dump the bytes regardless of the block state.

unsafe { handler.write(&mut block, 2000).unwrap() };

// You can also safe write using atomic states (States from 0 to 49 are reserved and will fail if tried)

handler.write_if(&mut block, 2000, MyCustomState).unwrap();
// or
handler.write_transition(&mut block, 2000, TargetState, MyCustomState, Ordering::Whatever).unwrap();

// Custom states can be any u32 integer from 49 and beyond.
// The target state can be any state besides FREE/ACKED/COALESCING.

// Read a copy of the data back

let data = handler.read_copy(&block).unwrap();
println!("{data}");

// You can also read type guarded references directly.

let rt_ref = handler.read(&block).unwrap(); // reference to T
let rt_mut_ref = handler.read_mut(&block).unwrap(); // mutable reference to T

// Free it

handler.free(block);

// Kill the arena

unsafe { handler.die().unwrap() }
```