/*
 * Copyright (C) 2026
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * RocketDrag - directional (anisotropic) aerodynamic drag for a rocket-shaped
 * body.  See include/RocketDrag.hh for the model and the SDF parameters.
 */

#include "RocketDrag.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace sim;
using namespace gz::sim::systems;

class gz::sim::systems::RocketDrag::Impl
{
  /// \brief Initialise from the model entity and the plugin SDF block.
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Apply the cross-flow drag wrench for this step.
  public: void Update(EntityComponentManager &_ecm);

  public: Model model{kNullEntity};
  public: Entity linkEntity{kNullEntity};

  /// Nose-on drag coefficient (dimensionless, referenced to `areaAxial`).
  public: double cdAxial = 0.057;
  /// Broadside (cross-flow) drag coefficient referenced to `areaNormal`.
  public: double cdNormal = 1.0;
  /// Reference area for the axial term (m^2).
  public: double areaAxial = 0.0095;
  /// Reference area for the cross-flow term (m^2).
  public: double areaNormal = 0.0552;
  /// Air density (kg/m^3).
  public: double rho = 1.2041;
  /// Below this speed the drag force is zero (avoids hover jitter).
  public: double minSpeed = 0.10;

  public: bool validConfig{false};
  public: bool initialized{false};
  public: sdf::ElementPtr sdfConfig;
};

//////////////////////////////////////////////////
void RocketDrag::Impl::Load(const EntityComponentManager &_ecm,
                            const sdf::ElementPtr &_sdf)
{
  if (!this->model.Valid(_ecm))
  {
    gzerr << "The RocketDrag system should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->cdAxial = _sdf->Get<double>("cd_axial", this->cdAxial).first;
  this->cdNormal = _sdf->Get<double>("cd_normal", this->cdNormal).first;
  this->areaAxial = _sdf->Get<double>("area_axial", this->areaAxial).first;
  this->areaNormal = _sdf->Get<double>("area_normal", this->areaNormal).first;
  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->minSpeed = _sdf->Get<double>("min_speed", this->minSpeed).first;

  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "The RocketDrag system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }
  const auto linkName = _sdf->Get<std::string>("link_name");
  const auto entities =
      entitiesFromScopedName(linkName, _ecm, this->model.Entity());
  if (entities.empty())
  {
    gzerr << "Link with name[" << linkName << "] not found. "
          << "The RocketDrag will not generate forces\n";
    this->validConfig = false;
    return;
  }
  this->linkEntity = *entities.begin();
  if (!_ecm.EntityHasComponentType(this->linkEntity, components::Link::typeId))
  {
    gzerr << "Entity with name[" << linkName << "] is not a link\n";
    this->validConfig = false;
    return;
  }

  gzmsg << "RocketDrag: link=" << linkName
        << " cd_axial=" << this->cdAxial << " S_ax=" << this->areaAxial
        << " cd_normal=" << this->cdNormal << " S_n=" << this->areaNormal
        << " rho=" << this->rho << std::endl;
  this->validConfig = true;
}

//////////////////////////////////////////////////
void RocketDrag::Impl::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("RocketDrag::Update");

  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);
  if (!worldLinVel || !worldPose)
    return;

  const auto &pose = worldPose->Data();
  const auto vel = worldLinVel->Data();
  if (vel.Length() <= this->minSpeed)
    return;

  // Nose axis in the world frame (model +Z).
  const auto bodyAxis = pose.Rot().RotateVector(math::Vector3d::UnitZ);

  // Decompose the velocity into the body-axis (axial) and perpendicular parts.
  const double vAx = bodyAxis.Dot(vel);
  const auto velNormal = vel - vAx * bodyAxis;

  const double q = 0.5 * this->rho;

  // Axial drag opposes the component along the nose.
  auto force = -q * this->cdAxial * this->areaAxial * vAx * std::abs(vAx) *
               bodyAxis;
  // Cross-flow drag opposes the component normal to the nose.
  const double vN = velNormal.Length();
  if (vN > 0.0)
    force += -q * this->cdNormal * this->areaNormal * vN * velNormal;

  if (!force.IsFinite())
    return;

  // Apply at the centre of mass: pure translational drag, no aero moment.
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, math::Vector3d::Zero);
}

//////////////////////////////////////////////////
RocketDrag::RocketDrag() : impl(std::make_unique<Impl>())
{
}

//////////////////////////////////////////////////
RocketDrag::~RocketDrag() = default;

//////////////////////////////////////////////////
void RocketDrag::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &, EventManager &)
{
  this->impl->model = Model(_entity);
  this->impl->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void RocketDrag::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("RocketDrag::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time, ignoring RocketDrag.\n";
    return;
  }

  if (!this->impl->initialized)
  {
    // Load here: not all entities exist when Configure() runs.
    this->impl->Load(_ecm, this->impl->sdfConfig);
    this->impl->initialized = true;
    if (this->impl->validConfig)
    {
      Link link(this->impl->linkEntity);
      link.EnableVelocityChecks(_ecm, true);
    }
  }

  if (_info.paused)
    return;

  if (this->impl->initialized && this->impl->validConfig)
    this->impl->Update(_ecm);
}

GZ_ADD_PLUGIN(RocketDrag,
              System,
              RocketDrag::ISystemConfigure,
              RocketDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RocketDrag, "gz::sim::systems::RocketDrag")
