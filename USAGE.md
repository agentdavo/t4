# Transputer Emulator Usage

The repository provides the **t4** emulator under `temu/`.  Build it on
Linux with:

```sh
make -C temu -f Makefile.linux
```

On Debian/Ubuntu systems install the `libnanomsg-dev` package to pull in
the networking library used by the emulator:

```sh
sudo apt-get install libnanomsg-dev
```

The resulting `t4` binary emulates a single T414/T800 processor.
Typical invocation booting a `.btl` image:

```sh
./t4 -ss -se -su -sb program.btl
```

Options of interest:

* `-sb <file>` – boot the given binary.
* `-ss` – enable host services so standard C I/O works.
* `-su` – enable instruction profiling written to `profile`.
* `-sx` – enable execution tracing.

See `temu/README.md` for the full list of options.

The emulator understands programs converted by `transputer-netload`.
After compiling a C source with `transputer-tcc` link it to an ELF file
and convert it with:

```sh
transputer-netload prog.elf prog.btl
```

Running the resulting `prog.btl` with `t4` executes the program on the
emulated processor and prints any host output to the console.
