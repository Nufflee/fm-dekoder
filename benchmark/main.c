#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <io.h>
#include <fcntl.h>
#include <unistd.h>
#include "rtl-sdr.h"

#define MHZ(x) ((x)*1000 * 1000)
#define KHZ(x) ((x)*1000)

/*
Results on author's machine:
  [benchmark_set_center_freq] - Average time is 52.427032 ms, min = 49.646000 ms, max = 88.380600 ms, stddev = ±4.706351 ms
  [benchmark_set_sample_rate] - Average time is 115.002754 ms, min = 110.546402 ms, max = 146.712708 ms, stddev = ±5.392377 ms
*/

// WARNING: Defining DRY_RUN means that you will not actually run any meaningful benchmarks
//#define DRY_RUN

float timespec_diff_ms(struct timespec start, struct timespec end)
{
  return (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) / 1e6;
}

typedef struct Benchmark
{
  const char *name;
  void (*const func)(int i);
  int numRuns;
  float *times;
} Benchmark;

rtlsdr_dev_t *device;

// TODO: GCC does not inline these since they are function pointers although Clang does so it is possible.
// Is there a way to force GCC to do it too?
// See: https://godbolt.org/z/EfYTd76cG
// Turns out there's ultimately no significant time difference between inlined and non-inlined code here,
// and it's likely that GCC's optimizer simply cannot inline this no matter what.
__attribute__((always_inline)) static inline void benchmark_set_sample_rate(int i)
{
  rtlsdr_set_sample_rate(device, MHZ(i % 2 == 0 ? 1.024 : 3.2));
}

__attribute__((always_inline)) static inline void benchmark_set_center_freq(int i)
{
  rtlsdr_set_center_freq(device, MHZ(i % 2 == 0 ? 100 : 200));
}

int main()
{
  uint32_t deviceCount = rtlsdr_get_device_count();

  printf("Found %d device(s)\n", deviceCount);

  if (deviceCount == 0)
  {
    fprintf(stderr, "ERROR: You need a functioning RTL-SDR device to run this program!\n");
    exit(-1);
  }

  printf("Name of first device: %s\n", rtlsdr_get_device_name(0));

  char manufacturer[256];
  char product[256];
  char serial[256];
  assert(rtlsdr_get_device_usb_strings(0, manufacturer, product, serial) == 0);

  printf("Manufacturer: %s, Product: %s, Serial: %s\n", manufacturer, product, serial);

  assert(rtlsdr_open(&device, 0) == 0);

  assert(rtlsdr_set_sample_rate(device, KHZ(256)) == 0);
  assert(rtlsdr_set_center_freq(device, MHZ(69)) == 0);

  printf("Waiting 1 second for the PLL to lock...\n");
  sleep(1);

#define ARRAY_SIZE(array) sizeof(array) / sizeof(array[0])

  Benchmark benchmarks[] = {
      {
          .name = "benchmark_set_center_freq",
          .func = benchmark_set_center_freq,
          .numRuns = 200,
      },
      {
          .name = "benchmark_set_sample_rate",
          .func = benchmark_set_sample_rate,
          .numRuns = 100,
      }};

  // Run the benchamrks
  for (size_t i = 0; i < ARRAY_SIZE(benchmarks); i++)
  {
    Benchmark *benchmark = &benchmarks[i];
    float *times = malloc(benchmark->numRuns * sizeof(float));

    printf("Starting `%s`:\n", benchmark->name);

#ifndef DRY_RUN
    for (int run = 0; run < benchmark->numRuns; run++)
#else
    for (int run = 0; run < 1; run++)
#endif
    {
      struct timespec time_start, time_end;

      clock_gettime(CLOCK_MONOTONIC, &time_start);
      benchmark->func(i);
      clock_gettime(CLOCK_MONOTONIC, &time_end);

      times[run] = timespec_diff_ms(time_start, time_end);

      printf("\t[%s]\t %d/%d: %f ms\n", benchmark->name, run + 1, benchmark->numRuns, times[run]);
    }

    benchmark->times = times;
  }

  // Compute and report the statistics
  for (size_t i = 0; i < ARRAY_SIZE(benchmarks); i++)
  {
    Benchmark benchmark = benchmarks[i];

    float averageTime = 0;
    float maxTime = 0;
    float minTime = __FLT_MAX__;

    for (int j = 0; j < benchmark.numRuns; j++)
    {
      float time = benchmark.times[j];

      averageTime += time;

      if (time > maxTime)
      {
        maxTime = time;
      }

      if (time < minTime)
      {
        minTime = time;
      }
    }

    averageTime /= (float)benchmark.numRuns;

    float standardDev = 0;

    for (int j = 0; j < benchmark.numRuns; j++)
    {
      standardDev += powf(benchmark.times[j] - averageTime, 2);
    }

    standardDev = sqrtf(standardDev / benchmark.numRuns);

    printf("Results:\n");

    struct timespec resolution;
    clock_getres(CLOCK_MONOTONIC, &resolution);

    printf("\tTimer resolution: %ld ns\n", resolution.tv_nsec);

    // This is required for unicode output to work.
    _setmode(_fileno(stdout), _O_U16TEXT);
    wprintf(L"\t[%s] - Average time is %f ms, min = %f ms, max = %f ms, stddev = ±%f ms\n", benchmark.name, averageTime, minTime, maxTime, standardDev);

    free(benchmark.times);
    benchmark.times = NULL;
  }

  rtlsdr_close(device);
}