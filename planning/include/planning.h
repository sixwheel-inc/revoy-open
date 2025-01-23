#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct Plan_ {
  double setSpeed;
  double setSteer;
  uint32_t gridWidth;
  uint32_t gridHeight;
  double cellSizeX;
  double cellSizeY;
  double originX;
  double originY;
  char *gridData;
  size_t gridDataSize;
} Plan;

#define MAX_OCCUPANCY_NUM_BYTES 50000

typedef struct Perception_ {
  int64_t time;

  // an occupancy bitmap, 0 is unoccupied and 1 is occupied
  // the kth bit will correspond to:
  //    x = ( floor(k / numRows) * bits_per_meter ) + x_offset
  //    y = ( (k % numRows) * bits_per_meter ) + y_offset
  char occupancy[MAX_OCCUPANCY_NUM_BYTES];

  // map size and dimensions
  uint32_t numBytes;
  uint32_t numBytesInRow;
  // N = numBytes / numBytesInRow
  // M = numBytesInRow * 8

  // scale bits to match world
  float bitsPerMeter;

  // offset grid into correct location
  float offsetX; // meters
  float offsetY; // meters

  // no orientation is specified, grid is assumed to be aligned w/ Ego

} Perception;

Perception MakeEmptyPerception();

Plan GetNextPlan(const Perception *perception);
void StartPlanning();
void EndPlanning();

#ifdef __cplusplus
} // extern "C"
#endif
