/*
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
*/

#ifndef ROCKETDRAG_HH_
#define ROCKETDRAG_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Directional (anisotropic) aerodynamic drag for a rocket-shaped body.
///
/// Gazebo's built-in LiftDrag system uses `cd = cda * alpha`, i.e. zero drag at
/// zero angle of attack, which cannot represent a slender vehicle: a bullet
/// body has a small drag coefficient nose-on and a much larger one broadside.
/// This system applies the cross-flow model instead:
///
///   F = -0.5 * rho * ( cd_ax * S_ax * v_ax * |v_ax| * u_hat
///                    + cd_n  * S_n  * v_n  * |v_n| )
///
/// where `u_hat` is the body (nose) axis of the target link.
///
/// ## System Parameters:
///
///   `<link_name>` The link the drag is applied to.  Required.
///
///   `<cd_axial>` Nose-on drag coefficient.  Default: 0.057.
///
///   `<area_axial>` Reference area for the axial term, m^2.  Default: 0.0095.
///
///   `<cd_normal>` Broadside (cross-flow) drag coefficient.  Default: 1.0.
///
///   `<area_normal>` Reference area for the cross-flow term, m^2.
///   Default: 0.0552.
///
///   `<air_density>` Air density in kg/m^3.  Default: 1.2041.
///
///   `<min_speed>` Speed below which no drag is applied, m/s.
///   Default: 0.10.
///
/// The force is applied at the link's centre of mass, so v1 adds pure
/// translational drag with no aerodynamic moment.
class RocketDrag :
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure
{
  /// \brief Destructor
  public: virtual ~RocketDrag();

  /// \brief Constructor
  public: RocketDrag();

  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) final;

  /// \internal
  /// \brief Private implementation
  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // ROCKETDRAG_HH_
