// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <jolt_rust_cpp/src/misc.h>

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>

#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// We're also using STL classes in this example
using namespace std;

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{
  // Format the message
  va_list list;
  va_start(list, inFMT);
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), inFMT, list);
  va_end(list);

  // Print to the TTY
  cout << buffer << endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
  // Print to the TTY
  cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << endl;

  // Breakpoint
  return true;
};

#endif // JPH_ENABLE_ASSERTS

namespace jolt_rust_cpp {

// Program entry point
// max_num_bodies is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
// Note: This value is low because this is a simple test. For a real project use something in the order of 65536.

  SimSystem::SimSystem(uint32_t max_num_bodies) {
    init(max_num_bodies);
  }

  int64_t SimSystem::init(uint32_t max_num_bodies)
  {
    // Register allocation hook. In this example we'll just let Jolt use malloc / free but you can override these if you want (see Memory.h).
    // This needs to be done before any other Jolt function is called.
    RegisterDefaultAllocator();

    // Install trace and assert callbacks
    Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)

    // Create a factory, this class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data.
    // It is not directly used in this example but still required.
    Factory::sInstance = new Factory();

    // Register all physics types with the factory and install their collision handlers with the CollisionDispatch class.
    // If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function.
    // If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
    RegisterTypes();

    // We need a temp allocator for temporary allocations during the physics update. We're
    // pre-allocating 10 MB to avoid having to do allocations during the physics update.
    // B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
    // If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
    // malloc / free.
    temp_allocator = new TempAllocatorImpl(10 * 1024 * 1024);

    // We need a job system that will execute physics jobs on multiple threads. Typically
    // you would implement the JobSystem interface yourself and let Jolt Physics run on top
    // of your own job scheduler. JobSystemThreadPool is an example implementation.
    job_system = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

    // This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
    const uint cNumBodyMutexes = 0;

    // This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
    // body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
    // too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
    // Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
    const uint cMaxBodyPairs = 1024;

    // This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
    // number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
    // Note: This value is low because this is a simple test. For a real project use something in the order of 10240.
    const uint cMaxContactConstraints = 1024;

    // Now we can create the actual physics system.
    // PhysicsSystem physics_system;
    physics_system = new PhysicsSystem();
    physics_system->Init(max_num_bodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    // make the z axis the vertical axis
    physics_system->SetGravity(RVec3(0.0, 0.0, -1.0));

    // A body activation listener gets notified when bodies activate and go to sleep
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    physics_system->SetBodyActivationListener(&body_activation_listener);

    // A contact listener gets notified when bodies (are about to) collide, and when they separate again.
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    physics_system->SetContactListener(&contact_listener);

    // The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
    // variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
    auto& body_interface = physics_system->GetBodyInterface();

    // Next we can create a rigid body to serve as the floor, we make a large box
    // Create the settings for the collision volume (the shape).
    // Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
    BoxShapeSettings floor_shape_settings(Vec3(100.0f, 100.0f, 1.0f));
    floor_shape_settings.SetEmbedded(); // A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.

    // Create the shape
    ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
    ShapeRefC floor_shape = floor_shape_result.Get(); // We don't expect an error here, but you can check floor_shape_result for HasError() / GetError()

    // Create the settings for the body itself. Note that here you can also set other properties like the restitution / friction.
    BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, 0.0_r, -1.0_r), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);

    // Create the actual rigid body
    Body *floor = body_interface.CreateBody(floor_settings); // Note that if we run out of bodies this can return nullptr

    floor_id = floor->GetID();
    // Add it to the world
    body_interface.AddBody(floor_id, EActivation::DontActivate);

    // Now create a dynamic body to bounce on the floor
    // Note that this uses the shorthand version of creating and adding a body to the world
    BodyCreationSettings sphere_settings(new SphereShape(0.5f), RVec3(0.0_r, 0.0_r, 2.0_r), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
    sphere_id = body_interface.CreateAndAddBody(sphere_settings, EActivation::Activate);

    // Now you can interact with the dynamic body, in this case we're going to give it a velocity.
    // (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
    body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, 0.0f, -1.0f));

    // create the car
    {
      const float wheel_radius = 0.3f;
      const float wheel_width = 0.1f;
      const float half_vehicle_length = 2.0f;
      const float half_vehicle_width = 0.9f;
      const float half_vehicle_height = 0.2f;

      // Create collision testers
      mTesters[0] = new VehicleCollisionTesterRay(Layers::MOVING);
      mTesters[1] = new VehicleCollisionTesterCastSphere(Layers::MOVING, 0.5f * wheel_width);
      mTesters[2] = new VehicleCollisionTesterCastCylinder(Layers::MOVING);

      // TODO(lucasw) rotate the car so z is vertical axis, here it spawns sideways
      // Create vehicle body
      RVec3 position(0, 2, 3);
      RefConst<Shape> car_shape = OffsetCenterOfMassShapeSettings(Vec3(0, -half_vehicle_height, 0), new BoxShape(Vec3(half_vehicle_width, half_vehicle_height, half_vehicle_length))).Create().Get();
      BodyCreationSettings car_body_settings(car_shape, position, Quat::sRotation(Vec3::sAxisZ(), sInitialRollAngle), EMotionType::Dynamic, Layers::MOVING);
      car_body_settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
      car_body_settings.mMassPropertiesOverride.mMass = 1500.0f;
      auto mCarBody = body_interface.CreateBody(car_body_settings);

      car_id = mCarBody->GetID();
      body_interface.AddBody(car_id, EActivation::Activate);

      // Create vehicle constraint
      VehicleConstraintSettings vehicle;
      vehicle.mDrawConstraintSize = 0.1f;
      vehicle.mMaxPitchRollAngle = sMaxRollAngle;

      // Suspension direction
      Vec3 front_suspension_dir = Vec3(Tan(sFrontSuspensionSidewaysAngle), -1, Tan(sFrontSuspensionForwardAngle)).Normalized();
      Vec3 front_steering_axis = Vec3(-Tan(sFrontKingPinAngle), 1, -Tan(sFrontCasterAngle)).Normalized();
      Vec3 front_wheel_up = Vec3(Sin(sFrontCamber), Cos(sFrontCamber), 0);
      Vec3 front_wheel_forward = Vec3(-Sin(sFrontToe), 0, Cos(sFrontToe));
      Vec3 rear_suspension_dir = Vec3(Tan(sRearSuspensionSidewaysAngle), -1, Tan(sRearSuspensionForwardAngle)).Normalized();
      Vec3 rear_steering_axis = Vec3(-Tan(sRearKingPinAngle), 1, -Tan(sRearCasterAngle)).Normalized();
      Vec3 rear_wheel_up = Vec3(Sin(sRearCamber), Cos(sRearCamber), 0);
      Vec3 rear_wheel_forward = Vec3(-Sin(sRearToe), 0, Cos(sRearToe));
      Vec3 flip_x(-1, 1, 1);

      // Wheels, left front
      WheelSettingsWV *w1 = new WheelSettingsWV;
      w1->mPosition = Vec3(
          half_vehicle_width,
          -0.9f * half_vehicle_height,
          half_vehicle_length - 2.0f * wheel_radius);
      w1->mSuspensionDirection = front_suspension_dir;
      w1->mSteeringAxis = front_steering_axis;
      w1->mWheelUp = front_wheel_up;
      w1->mWheelForward = front_wheel_forward;
      w1->mSuspensionMinLength = sFrontSuspensionMinLength;
      w1->mSuspensionMaxLength = sFrontSuspensionMaxLength;
      w1->mSuspensionSpring.mFrequency = sFrontSuspensionFrequency;
      w1->mSuspensionSpring.mDamping = sFrontSuspensionDamping;
      w1->mMaxSteerAngle = sMaxSteeringAngle;
      w1->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

      // Right front
      WheelSettingsWV *w2 = new WheelSettingsWV;
      w2->mPosition = Vec3(
          -half_vehicle_width,
          -0.9f * half_vehicle_height,
          half_vehicle_length - 2.0f * wheel_radius);
      w2->mSuspensionDirection = flip_x * front_suspension_dir;
      w2->mSteeringAxis = flip_x * front_steering_axis;
      w2->mWheelUp = flip_x * front_wheel_up;
      w2->mWheelForward = flip_x * front_wheel_forward;
      w2->mSuspensionMinLength = sFrontSuspensionMinLength;
      w2->mSuspensionMaxLength = sFrontSuspensionMaxLength;
      w2->mSuspensionSpring.mFrequency = sFrontSuspensionFrequency;
      w2->mSuspensionSpring.mDamping = sFrontSuspensionDamping;
      w2->mMaxSteerAngle = sMaxSteeringAngle;
      w2->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

      // Left rear
      WheelSettingsWV *w3 = new WheelSettingsWV;
      w3->mPosition = Vec3(
          half_vehicle_width,
          -0.9f * half_vehicle_height,
          -half_vehicle_length + 2.0f * wheel_radius);
      w3->mSuspensionDirection = rear_suspension_dir;
      w3->mSteeringAxis = rear_steering_axis;
      w3->mWheelUp = rear_wheel_up;
      w3->mWheelForward = rear_wheel_forward;
      w3->mSuspensionMinLength = sRearSuspensionMinLength;
      w3->mSuspensionMaxLength = sRearSuspensionMaxLength;
      w3->mSuspensionSpring.mFrequency = sRearSuspensionFrequency;
      w3->mSuspensionSpring.mDamping = sRearSuspensionDamping;
      w3->mMaxSteerAngle = 0.0f;

      // Right rear
      WheelSettingsWV *w4 = new WheelSettingsWV;
      w4->mPosition = Vec3(
          -half_vehicle_width,
          -0.9f * half_vehicle_height,
          -half_vehicle_length + 2.0f * wheel_radius);
      w4->mSuspensionDirection = flip_x * rear_suspension_dir;
      w4->mSteeringAxis = flip_x * rear_steering_axis;
      w4->mWheelUp = flip_x * rear_wheel_up;
      w4->mWheelForward = flip_x * rear_wheel_forward;
      w4->mSuspensionMinLength = sRearSuspensionMinLength;
      w4->mSuspensionMaxLength = sRearSuspensionMaxLength;
      w4->mSuspensionSpring.mFrequency = sRearSuspensionFrequency;
      w4->mSuspensionSpring.mDamping = sRearSuspensionDamping;
      w4->mMaxSteerAngle = 0.0f;

      vehicle.mWheels = { w1, w2, w3, w4 };

      for (WheelSettings *w : vehicle.mWheels)
      {
        w->mRadius = wheel_radius;
        w->mWidth = wheel_width;
      }

      WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
      vehicle.mController = controller;

      // Differential
      controller->mDifferentials.resize(sFourWheelDrive? 2 : 1);
      controller->mDifferentials[0].mLeftWheel = 0;
      controller->mDifferentials[0].mRightWheel = 1;
      if (sFourWheelDrive)
      {
        controller->mDifferentials[1].mLeftWheel = 2;
        controller->mDifferentials[1].mRightWheel = 3;

        // Split engine torque
        controller->mDifferentials[0].mEngineTorqueRatio = controller->mDifferentials[1].mEngineTorqueRatio = 0.5f;
      }

      // Anti rollbars
      if (sAntiRollbar)
      {
        vehicle.mAntiRollBars.resize(2);
        vehicle.mAntiRollBars[0].mLeftWheel = 0;
        vehicle.mAntiRollBars[0].mRightWheel = 1;
        vehicle.mAntiRollBars[1].mLeftWheel = 2;
        vehicle.mAntiRollBars[1].mRightWheel = 3;
      }

      mVehicleConstraint = new VehicleConstraint(*mCarBody, vehicle);

      // The vehicle settings were tweaked with a buggy implementation of the longitudinal tire impulses, this meant that PhysicsSettings::mNumVelocitySteps times more impulse
      // could be applied than intended. To keep the behavior of the vehicle the same we increase the max longitudinal impulse by the same factor. In a future version the vehicle
      // will be retweaked.
      static_cast<WheeledVehicleController *>(mVehicleConstraint->GetController())->SetTireMaxImpulseCallback(
          [](uint, float &outLongitudinalImpulse, float &outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float, float, float)
          {
          outLongitudinalImpulse = 10.0f * inLongitudinalFriction * inSuspensionImpulse;
          outLateralImpulse = inLateralFriction * inSuspensionImpulse;
          });

      physics_system->AddConstraint(mVehicleConstraint);
      // physics_system->AddStepListener(mVehicleConstraint);
    }


    // Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
    // You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
    // Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
    physics_system->OptimizeBroadPhase();

    // Now we're ready to simulate the body, keep simulating until it goes to sleep
    // while (body_interface.IsActive(sphere_id))

    return 0;
  }

  void SimSystem::close() {
    cout << "close " << step << endl;
    auto& body_interface = physics_system->GetBodyInterface();
    // Remove the sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
    body_interface.RemoveBody(sphere_id);

    // Destroy the sphere. After this the sphere ID is no longer valid.
    body_interface.DestroyBody(sphere_id);

    // Remove and destroy the floor
    body_interface.RemoveBody(floor_id);
    body_interface.DestroyBody(floor_id);

    // Unregisters all types with the factory and cleans up the default material
    UnregisterTypes();

    // Destroy the factory
    delete Factory::sInstance;
    Factory::sInstance = nullptr;
  }

  void SimSystem::update() {
    // cout << "update " << step << endl;
    auto& body_interface = physics_system->GetBodyInterface();

    // Output current position and velocity of the sphere
    RVec3 position = body_interface.GetCenterOfMassPosition(car_id);
    Vec3 linear_vel;
    Vec3 angular_vel;
    body_interface.GetLinearAndAngularVelocity(car_id, linear_vel, angular_vel);
    cout << "Step " << step
      << ": Position = (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ()
      << "), Linear Velocity = (" << linear_vel.GetX() << ", " << linear_vel.GetY() << ", " << linear_vel.GetZ() << ")"
      << "), Angular Velocity = (" << angular_vel.GetX() << ", " << angular_vel.GetY() << ", " << angular_vel.GetZ() << ")"
      << endl;

    //
    // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
    const int cCollisionSteps = 1;

    // Step the world
    physics_system->Update(cDeltaTime, cCollisionSteps, temp_allocator, job_system);

    ++step;
  }

  std::unique_ptr<SimSystem> new_sim_system(uint32_t max_num_bodies) {
    return std::make_unique<SimSystem>(max_num_bodies);
  }

} // namespace jolt_rust_cpp
