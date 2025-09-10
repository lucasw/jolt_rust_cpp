#include <cstdint>
#include <memory>

using namespace std;
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Core/RTTI.h>

using namespace JPH;
using namespace JPH::literals;

#include <jolt_rust_cpp/src/Layers.h>

namespace jolt_rust_cpp {
  class SimSystem {
    public:
      SimSystem();

      uint64_t value = 0;

      void setValue(uint64_t value);
      uint64_t getValue() const;

      // BPLayerInterfaceImpl  mBroadPhaseLayerInterface;  // The broadphase layer interface that maps object layers to broadphase layers
      // ObjectVsBroadPhaseLayerFilterImpl mObjectVsBroadPhaseLayerFilter;  // Class that filters object vs broadphase layers
      // ObjectLayerPairFilterImpl mObjectVsObjectLayerFilter;  // Class that filters object vs object layers

      PhysicsSystem * mPhysicsSystem = nullptr;
      BodyInterface * mBodyInterface = nullptr;
      Body& CreateFloor(float inSize = 200.0f);
      JobSystem * mJobSystem = nullptr;
      // DebugRenderer * mDebugRenderer = nullptr;
      TempAllocator * mTempAllocator = nullptr;
  };

  std::unique_ptr<SimSystem> new_sim_system();
} // namespace vehicle
