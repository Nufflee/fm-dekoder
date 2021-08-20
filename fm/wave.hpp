#pragma once

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <cstring>

enum class WaveFormat : uint16_t
{
  PCM = 0x0001
};

template <class T>
static void write_bytes(std::ofstream &stream, T value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

static void write_bytes(std::ofstream &stream, const char *value)
{
  stream.write(value, sizeof(char) * strlen(value));
}

template <class T>
class Wave
{
private:
  WaveFormat format;
  uint16_t channelCount;
  uint32_t bytesPerSample;
  uint32_t sampleRate;
  uint32_t sampleCount;
  const T *samples;

public:
  Wave(WaveFormat format, uint16_t channelCount, uint32_t sampleRate, uint32_t sampleCount)
      : format(format), channelCount(channelCount), bytesPerSample(sizeof(T)), sampleRate(sampleRate), sampleCount(sampleCount)
  {
  }

  void set_data(const T *samples)
  {
    this->samples = samples;
  }

  bool write_to_file(const char *path) const
  {
    std::ofstream file;

    file.open(path, std::ios::out | std::ios::binary);

    if (!file.is_open())
    {
      return false;
    }

    bool paddingRequired = false;

    // The WAVE standard requires a padding byte at the end of the data chunk if
    // the number of samples is odd
    if (sampleCount % 2 != 0)
    {
      paddingRequired = true;
    }

    uint32_t dataSize = channelCount * sampleCount * bytesPerSample;

    // Master Chunk
    {
      // Chunk ID
      write_bytes(file, "RIFF");

      // Chunk size
      uint32_t chunkSize = 4 + (8 + 16) + (8 + dataSize + (int)paddingRequired);
      write_bytes(file, chunkSize);

      file.write("WAVE", 4);
    }

    // Format Chunk
    {
      // Chunk ID
      write_bytes(file, "fmt ");

      // Chunk size
      uint32_t chunkSize = 16;
      write_bytes(file, chunkSize);

      // Format tag
      write_bytes(file, format);

      write_bytes(file, channelCount);

      write_bytes(file, sampleRate);

      // Average bytes per second
      uint32_t bytesPerSecond = channelCount * sampleRate * bytesPerSample;
      write_bytes(file, bytesPerSecond);

      uint16_t blockAlign = channelCount * bytesPerSample;
      write_bytes(file, blockAlign);

      uint16_t bitsPerSample = 8 * bytesPerSample;
      write_bytes(file, bitsPerSample);
    }

    // Data chunk
    {
      // Chunk ID
      write_bytes(file, "data");

      // Chunk size
      write_bytes(file, dataSize);

      file.write(reinterpret_cast<const char *>(samples), dataSize);

      if (paddingRequired)
      {
        write_bytes<char>(file, 0);
      }
    }

    return true;
  }
};