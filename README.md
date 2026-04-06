# AtomicMatrix

[![Status](https://img.shields.io/badge/status-Work%20in%20Progress-orange.svg)]()
[![Language](https://img.shields.io/badge/language-Rust-red.svg)]()

AtomicMatrix is a high-performance, ultra low latency IPC framework designed to enable true \
lock-free communication between independent threads.

Using SHM and Atomic objects at the core layout of the matrix, and providing a high level API \
for utilization, it aims to provide hardware performance by bypassing standard IPC and kernel \
context switching processes, and implementing a zero copy methodology to ensure no memory is \
consumed lightly.

---

> ⚠️ **Development Status:** This project is under active development.
> Core memory layouts and synchronization primitives are subjective to change.
>
> For an in-depth explanation of the architecture, please reffer to the arch folder.

---

## System Architecture & Design

The AtomicMatrix is built around 3 tiers of operation:

1. AtomicMatrix

- Stands as a single source of "truth" for all participant modules of the matrix. It is architectured \
around the **Two-Level Segregated Fit (TLSF)** principle for memory block organization, as well as \
providing all the necessary low level operations to manipulate the matrix through Atomic Primitives. \
It also holds the inbox structure that each module will use to read or leave messages to others.

2. MatrixHandler

- Acts as the High-Level API to communicate with the matrix. It encapsulates a lot of complex procedures \
that can be executed inside the matrix provided features, like converting higher order primitives into \
binaries and writing these into the memory block, notifying other modules inboxes of new messages, and \
so on. 

- This handler also provides a method that allow it to safely be copied to independent threads, \
giving only a pointer to the matrix, and mapping it to the local memory scope, rendering the matrix a \
Singleton struct throughout all the threads that participate.

3. Policy Constraints

- This module provides policy contraints for messages to be executed before publishing to the matrix. It \
should abstract a lot of concepts such as security (encryption, identification, encoding, padding), memory \
(size limitation, time to live, cleaning procedures), and so on.

This entire design is conceived to enable a non-block data structure that allows for secure, fast, low memory \
footprint communication with true asynchronicity, while still allowing for deep flexibility on how this \
communication is handled in the high level.

## Core Features

### Monolithic SHM

SHM initialization is handled during startup once and subsequent callers can register to the same address \
space using the refference from the main caller, ensuring all modules have access to the same dataplane and \
granting a zero-copy data sharing.

### Lock-Free synchronization

The use of a Singleton matrix refference, atomic primitives, and TLSF block separation between messages \
allow multiple threads to read/write simultaneously to the same address space, without ever catching a \
racing condition with eachother, or having to use OS-level locks.

### In depth customization

Through policy constraints, the structure allows the caller to customize communication between modules as \
they see fit.

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