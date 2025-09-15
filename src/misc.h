#include <cstdint>
#include <memory>

namespace jolt_rust_cpp {
  class SimSystem {
    public:
      SimSystem(uint32_t max_num_bodies);
      int64_t init(uint32_t max_num_bodies);
  };

  std::unique_ptr<SimSystem> new_sim_system(uint32_t max_num_bodies);
} // namespace jolt_rust_cpp
