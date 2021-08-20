#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <io.h>
#include <fcntl.h>
#include "rtl-sdr.h"

#define MHZ(x) ((x)*1000 * 1000)
#define KHZ(x) ((x)*1000)

/*
Frequency switch:
  now: ~60 ms, ±5 ms stddev

  earlier: ~50 ms, ±3 ms stddev
*/

float timespec_diff_ms(struct timespec start, struct timespec end)
{
  return (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) / 1e6;
}

int main()
{
  rtlsdr_dev_t *device;

  uint32_t deviceCount = rtlsdr_get_device_count();

  printf("Found %d device(s)\n", deviceCount);

  if (deviceCount == 0)
  {
    fprintf(stderr, "ERROR: You need a functioning RTL-SDR device to run this program!\n");
    exit(-1);
  }

  printf("Name of 0th device: %s\n", rtlsdr_get_device_name(0));

  char manufacturer[256];
  char product[256];
  char serial[256];
  assert(rtlsdr_get_device_usb_strings(0, manufacturer, product, serial) == 0);

  printf("Manufacturer: %s, Product: %s, Serial: %s\n", manufacturer, product, serial);

  assert(rtlsdr_open(&device, 0) == 0);

  assert(rtlsdr_set_sample_rate(device, 3200000) == 0);

#define NUM_RUNS 200

  float times[NUM_RUNS];

  for (int i = 0; i < NUM_RUNS; i++)
  {
    struct timespec time_start, time_end;

    clock_gettime(CLOCK_MONOTONIC, &time_start);
    rtlsdr_set_center_freq(device, MHZ(100));
    clock_gettime(CLOCK_MONOTONIC, &time_end);

    times[i] = timespec_diff_ms(time_start, time_end);

    printf("%d/%d: %f ms\n", i + 1, NUM_RUNS, times[i]);
  }

  float averageTime = 0;
  float maxTime = 0;
  float minTime = 99999;

  for (int i = 0; i < NUM_RUNS; i++)
  {
    float time = times[i];

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

  averageTime /= (float)NUM_RUNS;

  float standardDev = 0;

  for (int i = 0; i < NUM_RUNS; i++)
  {
    standardDev += powf(times[i] - averageTime, 2);
  }

  standardDev = sqrtf(standardDev / NUM_RUNS);

  // This is required for unicode output to work.
  _setmode(_fileno(stdout), _O_U16TEXT);
  wprintf(L"Average time is %f ms, min = %f ms, max = %f ms, stddev = ±%f ms", averageTime, minTime, maxTime, standardDev);

  rtlsdr_close(device);
}