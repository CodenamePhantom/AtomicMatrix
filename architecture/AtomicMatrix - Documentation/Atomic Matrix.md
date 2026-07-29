### Table of contents
- Core
	- [Atomic Matrix](Core/Matrix.md)
	- [Handlers](Core/Handlers.md)
	- [Cartographer](Core/Cartographer.md)
- Internals
	- [Atomic Ringbuffer](Internals/AtomicRingbuffer.md)
	- [Atomic Array](Internals/AtomicArray.md)
- Extensive Library
	- [Looper](Extensive%20Lib/Looper.md)
	- [Fencer](Extensive%20Lib/Fencer.md)
	- [Guardian](Extensive%20Lib/Guardian.md)

## Objective

Atomic Matrix is a lock free memory arena based on Linux SHM protocol. It enables completelly different processes to access the same memory space concurrently, providing seamless IPC and shared state magement.

