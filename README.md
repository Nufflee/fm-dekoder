# fm-dekoder

fm-dekoder is a FM broadcast radio demodulator playground project. The goal is to build a somewhat robust and fast FM demodulator and explore the capabilities of the [RTL-SDR](https://www.rtl-sdr.com/) with the ultimate goal of building a frequency scanner.

Currently, in order to run this project, you will need to have a functional RTL-SDR dongle.

## Building

Currently the project only supports Windows but that will be changed in the future.

```bash
make fm
./fm.exe
```

## Benchmark

To explore the capabilities of the RTL-SDR for frequency scanning, I wrote a little benchmark located in `benchmark/`.

It currently only benchmarks the time required to change the center frequency.

### Building

```bash
make benchmark
./benchmark.exe
```