# AtomicMatrix

[![Status](https://img.shields.io/badge/status-Work%20in%20Progress-orange.svg)]()
[![Language](https://img.shields.io/badge/language-Rust-red.svg)]()

AtomicMatrix is a high-performance, ultra low latency IPC (Inter-Process Communication) framework designed to enable true lock-free communication between independent process and threads.

By utilizing Shared Memory (SHM) and Atomic objects at its core, and providing a high-level API for seamless integration, AtomicMatrix achieves hardware-level performance. It bypasses standard IPC overhead and kernel context switching by implementing a **zero-copy methodology**, ensuring optimal memory efficiency and near-zero latency.

---

> ⚠️ **Development Status:** This project is under active development.
> Core memory layouts and synchronization primitives are subjective to change.

---

## System Architecture & Design

The AtomicMatrix is built around three operational tiers:

1. AtomicMatrix

- The single source of truth for all participant modules. It is architected around the **Two-Level Segregated Fit (TLSF)** principle for memory block organization. It provides all the necessary low-level operations to manipulate memory through **Atomic Primitives**, managing an "inbox" structure where modules exchange messages with deterministic O(1) timing.

2. MatrixHandler

- Acts as the High-Level API for matrix interaction. It encapsulates complex procedures, such as:
    - Converting high-order primitives into binary formats.
    - Writing data into memory blocks with atomic guards.
    - Notifying participant inboxes of new messages.
- The handler is designed to be safely shared across independent threads. It maps the matrix into the local process address space while maintaining a **Singleton-like** structure across all participating threads.

3. Policy Constraints

- A specialized module that enforces rules upon messages before they are published to the matrix. It abstract complex cross-cutting concerns:
    - **Security:** Encryption, identification, and secure encoding.
    - **Memory Management:** Size limitations, Time-to-Live (TTL), and automated cleaning procedures.
    - **Validation**: Padding and schema enforcement.

## Core Features

- **Monolithic SHM:** SHM initialization occurs once during startup. Subsequent callers attach to the existing address space, ensuring all modules operate on the same data plane, enabling **Zero-Copy data sharing**.

- **Lock-Free synchronization:** By combining a global matrix reference, atomic primitives, and TLSF block separation, multiple threads can read and write simultaneously without race conditions or the need for OS-level Mutexes/Spinlocks.

- **Deep customization:** Through **Policy Constraints**, users can customize communication protocols between modules as they see fit, ensuring flexibility without sacrificing performance.

## In-depth Documentation

For a detailed explanation of the internal mechanics and memory safety guarantess, please refer to the *~docs/arch~* folder

## Roadmap

Official roadmap to be published.

## Building the Code

To compile the codebase and inspect the memory layout:

```bash
# Clone the repository
git clone [https://github.com/CodenamePhantom/atomic-matrix.git](https://github.com/yourusername/atomic-matrix.git)

# Navigate to the workspace
cd atomic-matrix

# Build with release optimizations (crucial for atomic ordering performance)
cargo build --release