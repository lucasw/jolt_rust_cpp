#ifndef MISC_H
#define MISC_H

#include <cstdint>
#include <memory>

using namespace std;
#include <Jolt/Jolt.h>
// #include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
/*
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Core/RTTI.h>
*/

namespace jolt_rust_cpp {
  class SimSystem {
    private:
      JPH::PhysicsSystem * physics_system = nullptr;
      JPH::TempAllocator * temp_allocator = nullptr;
      JPH::JobSystem * job_system = nullptr;
      // JPH::BodyInterface& body_interface = nullptr;
      /*
      // DebugRenderer * mDebugRenderer = nullptr;
      */

      JPH::BodyID sphere_id;
      JPH::BodyID floor_id;

      unsigned int step = 0;

      // We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
      const float cDeltaTime = 1.0f / 60.0f;

      void close();

    public:
      SimSystem(uint32_t max_num_bodies);
      int64_t init(uint32_t max_num_bodies);
      void update();
  };

  std::unique_ptr<SimSystem> new_sim_system(uint32_t max_num_bodies);
} // namespace jolt_rust_cpp

#endif  // MISC_H
