#include <cstdint>

#include <jolt_rust_cpp/src/vehicle.h>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

namespace jolt_rust_cpp {
  SimSystem::SimSystem() {
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
