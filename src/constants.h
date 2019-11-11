#ifndef KIVA_SIMULATION_SRC_CONSTANTS_H_
#define KIVA_SIMULATION_SRC_CONSTANTS_H_

#include <map>
#include <cfloat>

#include "common_types.h"

namespace ks {

const char * const kMapFileName = "../data/map_1.map";

const int kRobotCount = 300;

// WMS related.
const int kPendingMoveToOperationMissionLimit = 400;
const int kMissionGenerationIntervalMs = 3000;
const int kMinOperationTimeS = 20;
const int kMoveOutProb = 20;

// Scheduler related.
const int kScheduleIntervalMs = 1000;
const int kUpdateIntervalMs = 100;

// Map related.
const char kShelfOperationPoint = 'P';
const char kShelfStoragePoint = 'S';
const char kRestArea = 'R';
const int kXLimit = 81;
const int kYLimit = 101;
const int kShelfOperationPointCount = 320;
const int kShelfStoragePointCount = 3200;
const int kShelfCount = 3100;
const int kRestAreaCount = 152;

// SIPP related.
const double kDoubleInf = DBL_MAX / 2;

// Time related.
const TimePoint kEpoch = std::chrono::time_point<std::chrono::system_clock>{};

// Robot manager related.
const std::map<Direction, std::pair<int, int>> kDirectionToDelta =
    {{Direction::NORTH, {-1, 0}},
     {Direction::SOUTH, {1, 0}},
     {Direction::WEST, {0, -1}},
     {Direction::EAST, {0, 1}}};

const double kPi = 3.14159;

const std::map<Direction, double> kDirectionToRadian = {
    {Direction::NORTH, kPi / 2},
    {Direction::SOUTH, kPi / 2 * 3},
    {Direction::WEST, kPi},
    {Direction::EAST, 0}
};

// Simulator related.
const Location kInvalidLocation = {-1, -1};

const int kTurnDurationMs = 2000;
const int kAttachDetachDurationMs = 3000;
const int kForwardDurationMs = 1500;
const int kWaitDurationMs = 1000;

const char *const kRedisHostname = "127.0.0.1";
const int kRedisPort = 6379;
const char *const kRedisKey = "ks";
}


#endif