#include <stdint.h>
#include <stdio.h>

struct WaveFile
{
  const char *fileName;
  uint16_t format;
  uint16_t channelCount;
  uint32_t bytesPerSample;
  uint32_t samplesPerSecond;
  uint32_t sampleCount;
};

#define WAVE_FORMAT_PCM 0x0001

void write_wave_file(WaveFile *config, uint8_t *data)
{
  FILE *file = fopen(config->fileName, "w");

  bool paddingRequired = false;

  // master chunk
  {
    // chunk id
    fwrite("RIFF", 1, 4, file);

    if (config->sampleCount % 2 != 0)
    {
      paddingRequired = true;
    }

    int chunkSize = 4 + (8 + 16) + (8 + config->channelCount * config->sampleCount * config->bytesPerSample + (int)paddingRequired);
    fwrite((char *)&chunkSize, 4, 1, file);

    // wave id
    fwrite("WAVE", 1, 4, file);
  }

  // format chunk
  {
    // chunk id
    fwrite("fmt ", 1, 4, file);

    int chunkSize = 16;
    fwrite((char *)&chunkSize, 4, 1, file);

    uint16_t formatTag = WAVE_FORMAT_PCM;
    fwrite((char *)&formatTag, 2, 1, file);

    fwrite((char *)&config->channelCount, 2, 1, file);

    fwrite((char *)&config->samplesPerSecond, 4, 1, file);

    int bytesPerSecond = config->channelCount * config->samplesPerSecond * config->bytesPerSample;
    fwrite((char *)&bytesPerSecond, 4, 1, file);

    uint16_t blockAlign = config->channelCount * config->bytesPerSample;
    fwrite((char *)&blockAlign, 2, 1, file);

    uint16_t bitsPerSample = 8 * config->bytesPerSample;
    fwrite((char *)&bitsPerSample, 2, 1, file);
  }

  // data chunk
  {
    // chunk id
    fwrite("data", 4, 1, file);

    int chunkSize = config->channelCount * config->sampleCount * config->bytesPerSample;
    fwrite((char *)&chunkSize, 4, 1, file);

    /*
    float frequency = 1000;

    for (uint32_t i = 0; i < config->sampleCount; i++)
    {
      uint8_t sample = (uint8_t)(255.0f * (sinf(2 * M_PI * frequency * ((float)i / (float)config->samplesPerSecond)) + 1) / 2.0f);
      fwrite((char *)&sample, 1, 1, file);
    }
    */

    for (uint32_t i = 0; i < config->sampleCount; i++)
    {
      uint8_t sample = data[i];
      fwrite((char *)&sample, 1, 1, file);
    }

    if (paddingRequired)
    {
      fwrite("\0", 1, 1, file);
    }
  }

  fclose(file);
}