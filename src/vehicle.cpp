#include <cstdarg>
#include <cstdint>
#include <iostream>

// #include <thread>

#include <jolt_rust_cpp/src/vehicle.h>

/*
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/RegisterTypes.h>
*/

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

using namespace std;

namespace jolt_rust_cpp {

  static constexpr uint cNumBodies = 10240;
  static constexpr uint cNumBodyMutexes = 0; // Autodetect
  static constexpr uint cMaxBodyPairs = 65536;
  static constexpr uint cMaxContactConstraints = 20480;

  SimSystem::SimSystem() {
    RegisterDefaultAllocator();

    // Install trace and assert callbacks
    Trace = TraceImpl;

    JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
    // this fouls up - look at hello world example to debug
    // AssertFailed = AssertFailedImpl;

    // Create a factory, this class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data.
    // It is not directly used in this example but still required.
    Factory::sInstance = new Factory();

    // Register all physics types with the factory and install their collision handlers with the CollisionDispatch class.
    // If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function.
    // If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
    // RegisterTypes();

    // We need a temp allocator for temporary allocations during the physics update. We're
    // pre-allocating 10 MB to avoid having to do allocations during the physics update.
    // B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
    // If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
    // malloc / free.
    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);

    // We need a job system that will execute physics jobs on multiple threads. Typically
    // you would implement the JobSystem interface yourself and let Jolt Physics run on top
    // of your own job scheduler. JobSystemThreadPool is an example implementation.
    // this also fouls up
    // JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

    // Create physics system
    mPhysicsSystem = new PhysicsSystem();
    mPhysicsSystem->Init(cNumBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectVsObjectLayerFilter);

#if 0
    CreateFloor();

    TriangleList triangles;
    const int cNumSegments = 100;
    const float cLoopWidth = 20.0f;
    const float cLoopRadius = 20.0f;
    const float cLoopThickness = 0.5f;
    Vec3 prev_center = Vec3::sZero();
    Vec3 prev_center_bottom = Vec3::sZero();
    for (int i = 0; i < cNumSegments; ++i)
    {
      float angle = i * 2.0f * JPH_PI / (cNumSegments - 1);
      Vec3 radial(0, -Cos(angle), Sin(angle));
      Vec3 center = Vec3(-i * cLoopWidth / (cNumSegments - 1), cLoopRadius, cLoopRadius) + cLoopRadius * radial;
      Vec3 half_width(0.5f * cLoopWidth, 0, 0);
      Vec3 center_bottom = center + cLoopThickness * radial;
      if (i > 0)
      {
        // Top surface
        triangles.push_back(Triangle(prev_center + half_width, prev_center - half_width, center - half_width));
        triangles.push_back(Triangle(prev_center + half_width, center - half_width, center + half_width));

        // Bottom surface
        triangles.push_back(Triangle(prev_center_bottom + half_width, center_bottom - half_width, prev_center_bottom - half_width));
        triangles.push_back(Triangle(prev_center_bottom + half_width, center_bottom + half_width, center_bottom - half_width));

        // Sides
        triangles.push_back(Triangle(prev_center + half_width, center + half_width, prev_center_bottom + half_width));
        triangles.push_back(Triangle(prev_center_bottom + half_width, center + half_width, center_bottom + half_width));
        triangles.push_back(Triangle(prev_center - half_width, prev_center_bottom - half_width, center - half_width));
        triangles.push_back(Triangle(prev_center_bottom - half_width, center_bottom - half_width, center - half_width));
      }
      prev_center = center;
      prev_center_bottom = center_bottom;
    }
    MeshShapeSettings mesh(triangles);
    mesh.SetEmbedded();

    Body &loop = *mBodyInterface->CreateBody(BodyCreationSettings(&mesh, RVec3::sZero(), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
    loop.SetFriction(1.0f);
    mBodyInterface->AddBody(loop.GetID(), EActivation::Activate);
#endif
  }

  // this fouls up
#if 0
  SimSystem::CreateFloor(float inSize)
  {
    const float scale = GetWorldScale();

    Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(scale * Vec3(0.5f * inSize, 1.0f, 0.5f * inSize), 0.0f), RVec3(scale * Vec3(0.0f, -1.0f, 0.0f)), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
    mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);
    return floor;
  }
#endif

  void SimSystem::setValue(uint64_t value) {
    this->value = value;
  }

  uint64_t SimSystem::getValue() const {
    return value;
  }

  std::unique_ptr<SimSystem> new_sim_system() {
    return std::make_unique<SimSystem>();
  }
} // namespace vehicle
