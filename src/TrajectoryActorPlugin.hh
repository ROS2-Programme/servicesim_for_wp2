/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_
#define SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_

// This is just to get code to compile on both old (< 11) and new (>= 11)
// versions Gazebo libs.
// Not confirmed that code will run correctly with old Gazebo versions.
// Reference:
// https://github.com/osrf/gazebo/blob/gazebo11/Migration.md#gazebo-10x-to-110
#if GAZEBO_MAJOR_VERSION >= 11
#include <ignition/math/AxisAlignedBox.hh>
#define BBOX_TYPE ignition::math::AxisAlignedBox
#else
#include <ignition/math/Box.hh>
#define BBOX_TYPE ignition::math::Box
#endif

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

namespace servicesim
{
  class TrajectoryActorExtendedPlugin_Private;

  class GAZEBO_VISIBLE TrajectoryActorExtendedPlugin
      : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: TrajectoryActorExtendedPlugin();

    /// \brief Destructor
    public: ~TrajectoryActorExtendedPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
		sdf::ElementPtr _sdf) override;

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// \brief When user requests reset.
    private: void Reset() override;

    /// \brief Checks if there is an obstacle on the way.
    /// \return True if there is
    private: bool ObstacleOnTheWay() const;

    /// \brief Update target
    private: bool UpdateTarget();

    /// \internal
    private: TrajectoryActorExtendedPlugin_Private *dataPtr;

	protected:
		bool checkForModelChildLink( const gazebo::physics::WorldPtr &_world,
			const std::string &_szModel, const std::string &_szChild,
			const bool &_bDebug=false) const;

		bool loadDebugFlag( const sdf::ElementPtr &_sdf);

		unsigned int loadOptionalScaling(
			std::map< std::string, ignition::math::Vector3d> &_pScaleMapRet,
			std::map< std::string, ignition::math::Pose3d> &_pOffsetMapRet,
			const sdf::ElementPtr &_sdf, const bool &_bDebug=false);

		unsigned int rescaleAllLinkCollision(
			const gazebo::physics::LinkPtr &_link,
			const std::map< std::string, ignition::math::Vector3d> &_pScaleMap,
			const std::map< std::string, ignition::math::Pose3d> &_pOffsetMap,
			const bool &_bDebug=false);

		unsigned int rescaleActorSkeleton(
			const std::map< std::string,
				ignition::math::Vector3d> &_pScaleMapRet,
			const std::map< std::string,
				ignition::math::Pose3d> &_pOffsetMapRet,
			const bool &_bDebug=false);

	protected:
		bool testForCollisionNonActor( const std::string &_szMe,
			const gazebo::physics::ModelPtr &_model,
			const std::string &_szNameModel, const std::string &_szNameChild,
			const BBOX_TYPE &_bbReal,
			bool &_bbLastCollideNonActor, const bool &_bDebug) const;

		int testForCollisionActor( const std::string &_szMe,
			const gazebo::physics::ModelPtr &_model,
			const gazebo::physics::WorldPtr &_world,
			const BBOX_TYPE &_bbReal,
			bool &_bLastCollideActor,  const bool &_bDebug) const;

		int testModelForCollision( const std::string &_szMe,
			const unsigned int &_i, const gazebo::physics::ModelPtr &_model,
			const gazebo::physics::WorldPtr &_world,
			const BBOX_TYPE &_bbReal,
			bool &_bLastCollideNonActor, bool &_bLastCollideActor,
			const bool &_bDebug) const;

  };
}
#endif
