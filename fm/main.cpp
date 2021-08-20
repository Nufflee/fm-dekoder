#include <rtl-sdr.h>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#define _USE_MATH_DEFINES
#include <cmath>
#include <concepts>
#include <limits>
#include "wave.hpp"
#include "coeffs.h"

/*
  RTL2823U Datasheet: https://kechuang.org/r/303602
  RT820T* Partial Datasheet: https://www.rtl-sdr.com/wp-content/uploads/2013/04/R820T_datasheet-Non_R-20111130_unlocked1.pdf
  R820T2 Register Description: https://www.rtl-sdr.com/wp-content/uploads/2016/12/R820T2_Register_Description.pdf?ffccfa&ffccfa
*/

#define MHZ(x) ((x)*1000 * 1000)
#define KHZ(x) ((x)*1000)

// Redefinition of `M_PI` as a float because C++ assumes `M_PI` is a double which may slow down our float only computations
constexpr float PI = 3.14159265358979323846f;

constexpr uint32_t SAMPLE_RATE = KHZ(3200);

// This is a flooring modulo operation because libc's `fmodf` does a trunacting modulo which is
// not applicable here because we need the result to have the sign of the divisor.
// References:
//   - https://en.wikipedia.org/wiki/Modulo_operation
//   - It appears that this approach is ~2.2x faster than using fmodf: https://quick-bench.com/q/tRQBplshI4Rrkp5HsoaiuncD_Uw
template <std::floating_point T>
T mod_floor(T dividend, T divisor)
{
  return dividend - divisor * std::floor(dividend / divisor);
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

  assert(rtlsdr_open(&device, 0) == 0);
  assert(device);

  rtlsdr_set_center_freq(device, MHZ(102.5f));
  assert(rtlsdr_set_sample_rate(device, SAMPLE_RATE) == 0);

  rtlsdr_set_tuner_gain_mode(device, 1);
  assert(rtlsdr_set_tuner_gain(device, 496) == 0);

  printf("RTL-SDR device initialized!\n");

  int iqSamplesRead;

  // ALWAYS have to do this before calling read(_sync)
  rtlsdr_reset_buffer(device);

#define SAMPLE_LENGTH_SEC 5
#define SAMPLE_COUNT (SAMPLE_RATE * SAMPLE_LENGTH_SEC)
#define READ_BUFFER_SIZE (SAMPLE_COUNT * 2)

  uint8_t *iqReadBuffer = new uint8_t[READ_BUFFER_SIZE];

  printf("Acquiring %d IQ samples...\n", SAMPLE_COUNT);

  // Read data from the SDR
  assert(rtlsdr_read_sync(device, iqReadBuffer, READ_BUFFER_SIZE, &iqSamplesRead) == 0);
  assert(iqSamplesRead == READ_BUFFER_SIZE);

  float *iqReadBufferFloat = new float[READ_BUFFER_SIZE];

  printf("Preprocessing read samples...\n");

  // Map uint8_t[0, 255] -> float[-127.5, 127.5]
  for (uint32_t i = 0; i < READ_BUFFER_SIZE; i++)
  {
    iqReadBufferFloat[i] = (float)iqReadBuffer[i] - 127.5f;
  }

  printf("Demodulating FM...\n");

  float *angles = new float[SAMPLE_COUNT];

  // Compute the phase angle of each sample
  for (uint32_t i = 0; i < READ_BUFFER_SIZE; i += 2)
  {
    angles[i / 2] = atan2f(iqReadBufferFloat[i + 1], iqReadBufferFloat[i]);
  }

  float *rotations = new float[SAMPLE_COUNT - 1];

  // Compute the derivative of the phase angle for each sample
  for (uint32_t i = 0; i < SAMPLE_COUNT - 1; i++)
  {
    rotations[i] = angles[i + 1] - angles[i];

    // Map from [-2pi, 2pi] to [-pi, pi] centered at 0
    rotations[i] = mod_floor(rotations[i] + PI, 2 * PI) - PI;
  }

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define coeffs coeffs3200

  // Decimate to ~44.1 kHz
  uint32_t decimationFactor = roundf((float)SAMPLE_RATE / (float)KHZ(44.1));
  uint32_t decimatedSampleRate = SAMPLE_RATE / decimationFactor;
  uint32_t decimatedSampleCount = (SAMPLE_COUNT - 1) / decimationFactor;

  printf("Low-pass filtering and decimating...\n");

  printf("Decimation factor: %d, decimated sample rate: %d Hz\n", decimationFactor, decimatedSampleRate);

  float *decimatedRotations = new float[decimatedSampleCount];

  float decimatedMin = std::numeric_limits<float>::max();
  float decimatedMax = std::numeric_limits<float>::min();

  for (uint32_t i = 0; i < SAMPLE_COUNT - 1; i += decimationFactor)
  {
    if (i < ARRAY_SIZE(coeffs))
    {
      decimatedRotations[i] = rotations[i];

      continue;
    }

    float sum = 0;

    for (size_t j = 0; j < ARRAY_SIZE(coeffs); j++)
    {
      sum += rotations[i - j] * coeffs[j];
    }

    if (sum > decimatedMax)
    {
      decimatedMax = sum;
    }

    if (sum < decimatedMin)
    {
      decimatedMin = sum;
    }

    decimatedRotations[i / decimationFactor] = sum;
  }

  // Example:
  //   1 = [x, y]
  //   2 = [0, 255]
  //   0 + (value - x) * 255 / (y - x) = (value - x) / (y - x) * 255
  auto remap = [](float low1, float high1, float low2, float high2, float value) {
    return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
  };

  auto map_float = [&decimatedMin, &decimatedMax, &remap]<std::integral T>(float *floatBuffer, T *outputBuffer, size_t length) {
    for (uint32_t i = 0; i < length; i++)
    {
      outputBuffer[i] = roundf(remap(decimatedMin, decimatedMax, (float)std::numeric_limits<T>::min(), (float)std::numeric_limits<T>::max(), floatBuffer[i]));
    }
  };

  uint8_t *rawSamples = new uint8_t[SAMPLE_COUNT - 1];
  map_float(rotations, rawSamples, SAMPLE_COUNT - 1);

  uint8_t *decimatedSamples = new uint8_t[decimatedSampleCount];
  map_float(decimatedRotations, decimatedSamples, decimatedSampleCount);

  // Per, the WAVE standard 8-bit samples are always unsigned, but 16-bit samples are always signed.
  int16_t *rawSamples16 = new int16_t[SAMPLE_COUNT - 1];
  map_float(rotations, rawSamples16, SAMPLE_COUNT - 1);

  int16_t *decimatedSamples16 = new int16_t[decimatedSampleCount];
  map_float(decimatedRotations, decimatedSamples16, decimatedSampleCount);

  printf("Writing to WAVE files...\n");

  Wave<uint8_t> decimatedWave(WaveFormat::PCM, 1, decimatedSampleRate, decimatedSampleCount);

  decimatedWave.set_data(decimatedSamples);
  decimatedWave.write_to_file("decimated-audio.wav");

  Wave<uint8_t> rawWave(WaveFormat::PCM, 1, SAMPLE_RATE, SAMPLE_COUNT - 1);

  rawWave.set_data(rawSamples);
  rawWave.write_to_file("raw-audio.wav");

  Wave<int16_t> decimatedWave16(WaveFormat::PCM, 1, decimatedSampleRate, decimatedSampleCount);

  decimatedWave16.set_data(decimatedSamples16);
  decimatedWave16.write_to_file("decimated-audio16.wav");

  Wave<int16_t> rawWave16(WaveFormat::PCM, 1, SAMPLE_RATE, SAMPLE_COUNT - 1);

  rawWave16.set_data(rawSamples16);
  rawWave16.write_to_file("raw-audio16.wav");

  exit(0);

  // Write the raw samples to a file for further analysis in Python
  FILE *file = fopen("rotations", "w+");

  char buffer[16];

  int length = sprintf(buffer, "%d\n", SAMPLE_RATE);
  assert(length <= 16);

  fwrite(buffer, 1, length, file);

  for (uint32_t i = 0; i < SAMPLE_COUNT - 1; i++)
  {
    int length = sprintf(buffer, "%f\n", decimatedRotations[i]);
    assert(length <= 16);

    fwrite(buffer, 1, length, file);
  }

  fclose(file);

  printf("Analysis file written!\n");
}