#ifndef MISC_H
#define MISC_H

#include <cstdint>
#include <iostream>
#include <memory>

using namespace std;
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
  static constexpr JPH::ObjectLayer NON_MOVING = 0;
  static constexpr JPH::ObjectLayer MOVING = 1;
  static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
  virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
  {
    switch (inObject1)
    {
    case Layers::NON_MOVING:
      return inObject2 == Layers::MOVING; // Non moving only collides with moving
    case Layers::MOVING:
      return true; // Moving collides with everything
    default:
      JPH_ASSERT(false);
      return false;
    }
  }
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
  static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
  static constexpr JPH::BroadPhaseLayer MOVING(1);
  static constexpr uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
  BPLayerInterfaceImpl()
  {
    // Create a mapping table from object to broad phase layer
    mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
    mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
  }

  virtual uint GetNumBroadPhaseLayers() const override
  {
    return BroadPhaseLayers::NUM_LAYERS;
  }

  virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
  {
    JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
    return mObjectToBroadPhase[inLayer];
  }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  virtual const char * GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
  {
    switch ((JPH::BroadPhaseLayer::Type)inLayer)
    {
      case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:  return "NON_MOVING";
      case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:    return "MOVING";
      default: JPH_ASSERT(false); return "INVALID";
    }
  }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
  BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

  namespace jolt_rust_cpp {
    struct CVec3;
    struct CTf;
  }

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
  virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
  {
    switch (inLayer1)
    {
    case Layers::NON_MOVING:
      return inLayer2 == BroadPhaseLayers::MOVING;
    case Layers::MOVING:
      return true;
    default:
      JPH_ASSERT(false);
      return false;
    }
  }
};

// An example contact listener
class MyContactListener : public JPH::ContactListener
{
public:
  // See: ContactListener
  virtual ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
  {
    std::cout << "Contact validate callback" << endl;

    // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
    return ValidateResult::AcceptAllContactsForThisBodyPair;
  }

  virtual void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
  {
    std::cout << "A contact was added" << endl;
  }

  virtual void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
  {
    std::cout << "A contact was persisted" << endl;
  }

  virtual void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
  {
    std::cout << "A contact was removed" << endl;
  }
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
  virtual void OnBodyActivated(const JPH::BodyID &body_id, uint64 inBodyUserData) override
  {
    cout << "A body got activated " << body_id.GetIndex() << " " << static_cast<uint32_t>(body_id.GetSequenceNumber()) << endl;
  }

  virtual void OnBodyDeactivated(const JPH::BodyID &body_id, uint64 inBodyUserData) override
  {
    cout << "A body went to sleep " << body_id.GetIndex() << " " << static_cast<uint32_t>(body_id.GetSequenceNumber()) << endl;
  }
};

namespace jolt_rust_cpp {
  class SimSystem {
    private:
      JPH::PhysicsSystem * physics_system = nullptr;
      JPH::TempAllocator * temp_allocator = nullptr;
      JPH::JobSystem * job_system = nullptr;
      // JPH::BodyInterface& body_interface;
      // DebugRenderer * mDebugRenderer = nullptr;

      MyBodyActivationListener body_activation_listener;
      MyContactListener contact_listener;

      // Create mapping table from object layer to broadphase layer
      // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
      // Also have a look at BroadPhaseLayerInterfaceTable or BroadPhaseLayerInterfaceMask for a simpler interface.
      BPLayerInterfaceImpl broad_phase_layer_interface;

      // Create class that filters object vs broadphase layers
      // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
      // Also have a look at ObjectVsBroadPhaseLayerFilterTable or ObjectVsBroadPhaseLayerFilterMask for a simpler interface.
      ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;

      // Create class that filters object vs object layers
      // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
      // Also have a look at ObjectLayerPairFilterTable or ObjectLayerPairFilterMask for a simpler interface.
      ObjectLayerPairFilterImpl object_vs_object_layer_filter;

      JPH::BodyID sphere_id;
      JPH::BodyID floor_id;

      unsigned int step = 0;

      // We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
      const float cDeltaTime = 1.0f / 60.0f;

      // The car
      // TODO(lucasw) separate into another class
      static inline float     sInitialRollAngle = 0;
      static inline float     sMaxRollAngle = DegreesToRadians(60.0f);
      static inline float     sMaxSteeringAngle = DegreesToRadians(30.0f);
      static inline int       sCollisionMode = 2;
      static inline bool      sFourWheelDrive = false;
      static inline bool      sAntiRollbar = true;
      static inline bool      sLimitedSlipDifferentials = true;
      static inline bool      sOverrideGravity = false;         ///< If true, gravity is overridden to always oppose the ground normal
      static inline float     sMaxEngineTorque = 500.0f;
      static inline float     sClutchStrength = 10.0f;
      static inline float     sFrontCasterAngle = 0.0f;
      static inline float     sFrontKingPinAngle = 0.0f;
      static inline float     sFrontCamber = 0.0f;
      static inline float     sFrontToe = 0.0f;
      static inline float     sFrontSuspensionForwardAngle = 0.0f;
      static inline float     sFrontSuspensionSidewaysAngle = 0.0f;
      static inline float     sFrontSuspensionMinLength = 0.3f;
      static inline float     sFrontSuspensionMaxLength = 0.5f;
      static inline float     sFrontSuspensionFrequency = 1.5f;
      static inline float     sFrontSuspensionDamping = 0.5f;
      static inline float     sRearSuspensionForwardAngle = 0.0f;
      static inline float     sRearSuspensionSidewaysAngle = 0.0f;
      static inline float     sRearCasterAngle = 0.0f;
      static inline float     sRearKingPinAngle = 0.0f;
      static inline float     sRearCamber = 0.0f;
      static inline float     sRearToe = 0.0f;
      static inline float     sRearSuspensionMinLength = 0.3f;
      static inline float     sRearSuspensionMaxLength = 0.5f;
      static inline float     sRearSuspensionFrequency = 1.5f;
      static inline float     sRearSuspensionDamping = 0.5f;

      // Body * mCarBody;  ///< The vehicle
      JPH::BodyID car_id;
      Ref<VehicleConstraint> mVehicleConstraint;  ///< The vehicle constraint
      Ref<VehicleCollisionTester> mTesters[3];  ///< Collision testers for the wheel

    public:
      SimSystem(uint32_t max_num_bodies,
        jolt_rust_cpp::CVec3 floor_pos);
      int64_t init(uint32_t max_num_bodies,
        jolt_rust_cpp::CVec3 floor_pos);
      void pre_physics_update();
      CTf update();
      void close();

      // Player input
      float mForward = 0.0f;
      float mPreviousForward = 1.0f;  ///< Keeps track of last car direction so we know when to brake and when to accelerate
      float mRight = 0.0f;
      float mBrake = 0.0f;
      float mHandBrake = 0.0f;
  };

  std::unique_ptr<SimSystem> new_sim_system(uint32_t max_num_bodies,
      jolt_rust_cpp::CVec3 floor_pos);
} // namespace jolt_rust_cpp

#endif  // MISC_H
