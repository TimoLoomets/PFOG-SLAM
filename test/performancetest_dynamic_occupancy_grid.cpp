#include <benchmark/benchmark.h>
#include "dynamic_occupancy_grid.hpp"

static void BM_AccessInUninitialized(benchmark::State& state) {
  for (auto _ : state)
  {
    SLAM::DynamicOccupancyGrid grid;
    int reads = 0;
    for (float x = -10.0f; x < 10.0f; x += 0.01)
    {
        for (float y = -10.0f; y < 10.0f; y += 0.01)
        {
            grid.get_occupancy(x, y);
            ++reads;
        }
    }

    state.counters["Reads"] = reads;
  }
}

static void BM_SetValues(benchmark::State& state) {
  for (auto _ : state)
  {
    SLAM::DynamicOccupancyGrid grid;
    int writes = 0;
    for (float x = -10.0f; x < 10.0f; x += 0.01)
    {
        for (float y = -10.0f; y < 10.0f; y += 0.01)
        {
            grid.set_occupancy(x, y, 0.7f);
            ++writes;
        }
    }

    state.counters["Writes"] = writes;
  }
}

BENCHMARK(BM_AccessInUninitialized);
BENCHMARK(BM_SetValues);

BENCHMARK_MAIN();