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

#include <functional>
#include <math.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include "TrajectoryActorPlugin.hh"

using namespace gazebo;
using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::TrajectoryActorExtendedPlugin)

class servicesim::TrajectoryActorExtendedPlugin_Private
{
  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief List of targets
  public: std::vector<ignition::math::Pose3d> targets;

  /// \brief Current target index
  public: unsigned int currentTarget{0};

  /// \brief Radius in meters around target pose where we consider it was
  /// reached.
  public: double targetRadius{0.5};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.5};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Time of the last update.
  public: common::Time lastUpdate;

  /// \brief Time when the corner starts
  public: common::Time firstCornerUpdate;

  /// \brief List of models to avoid
  public: std::vector<std::string> obstacles;

  /// \brief Animation for corners
  public: common::PoseAnimation *cornerAnimation{nullptr};

  /// \brief Frequency in Hz to update
  public: double updateFreq{60};


  /// \brief Ignition communication node.
  public: ros::NodeHandle *rosnode_;

  /// \brief Publisher used to publish the messages.
  public: ros::Publisher pub_;

  public: std_msgs::Header hdrMsg;


  public:
    physics::ModelPtr model{ nullptr};
    physics::LinkPtr mainLink{ nullptr};

  public:
    bool bDebugOnLoad{ false};
    bool bDebugOnUpdate{ false};
    bool bDebugOnAdHoc{ false};

    bool bUseSkeleton4Collision{ false};

    int nSameLastTarget{ 0};
    int nSameTimeInt{ 0};
    int nIntraCycleMax{ 0};

    double dMaxLookAhead{ 5.0};

  public:
    ros::Publisher pub4LookAhead;

  public:
    static bool bFirstIter;

	static std::map< std::string, bool> pbLastCollideStatic;
	static std::map< std::string, bool> pbLastCollideActor;
};

bool servicesim::TrajectoryActorExtendedPlugin_Private::bFirstIter = true;
std::map< std::string, bool>
	servicesim::TrajectoryActorExtendedPlugin_Private::pbLastCollideStatic = {};
std::map< std::string, bool>
	servicesim::TrajectoryActorExtendedPlugin_Private::pbLastCollideActor = {};

/////////////////////////////////////////////////
TrajectoryActorExtendedPlugin::TrajectoryActorExtendedPlugin()
    : dataPtr(new TrajectoryActorExtendedPlugin_Private)
{
}

// ########################################
TrajectoryActorExtendedPlugin::~TrajectoryActorExtendedPlugin()
{
	this->dataPtr->pub_.shutdown();
	this->dataPtr->rosnode_->shutdown();
	delete( this->dataPtr->rosnode_);
	this->dataPtr->rosnode_ = NULL;
}

// ########################################
bool
TrajectoryActorExtendedPlugin::checkForModelChildLink(
	const physics::WorldPtr &_world, const std::string &_szModel,
	const std::string &_szChild, const bool &_bDebug) const
{
	assert( _world);

	auto _model = _world->ModelByName( _szModel);

	if( !_model) {
		gzwarn << "# c4MCL(): World has no model \"" << _szModel << "\""
			<< std::endl;
		return false;
	}

	if( _bDebug) {
		gzmsg << "# c4MCL(): Model \"" << _model->GetName()
			<< "\": GetChildCount() = " << _model->GetChildCount() << std::endl;
	}

	auto _child = _model->GetChild( _szChild);

	if( !_child) {
		gzwarn << "# c4MCL(): Model \"" << _model->GetName()
			<< "\" does not have child \"" << _szChild << "\"" << std::endl;
		return false;
	}

	if( !( _child->HasType( physics::Model::EntityType::LINK))) {
		gzwarn << "# c4MCL(): Child \"" << _szChild << "\" of model \""
			<< _szModel << "\" is not a LINK type." << std::endl;
		return false;
	}

	if( _bDebug) {
		auto _cLnk = boost::dynamic_pointer_cast< physics::Link>( _child);
		gzmsg << "# c4MCL(): " << _szModel << "." << _szChild << " BBox = "
			<< _cLnk->BoundingBox() << std::endl;
	}

	return true;
}

// ########################################
bool
TrajectoryActorExtendedPlugin::loadDebugFlag( const sdf::ElementPtr &_sdf) {
	if( !( _sdf->HasElement( "debug"))) {
		return false;
	}

    auto _el = _sdf->GetElement( "debug");

	this->dataPtr->bDebugOnLoad = _el->Get<bool>( "on_load");
	this->dataPtr->bDebugOnUpdate = _el->Get<bool>( "on_update");
	this->dataPtr->bDebugOnAdHoc = _el->Get<bool>( "on_adhoc");
	return true;
}

// ########################################
unsigned int
TrajectoryActorExtendedPlugin::loadOptionalScaling(
	std::map< std::string, ignition::math::Vector3d> &_pScaleMapRet,
	std::map< std::string, ignition::math::Pose3d> &_pOffsetMapRet,
	const sdf::ElementPtr &_sdf, const bool &_bDebug)
{
	unsigned int _nRet = 0;

	// Read in the collision scaling factors, if present

	if( !( _sdf->HasElement( "scaling"))) {
		if( _bDebug) {
			gzmsg << "# lOS(): No <scaling> element(s) in SDF." << std::endl;
		}
		return _nRet;
	}

	auto elem = _sdf->GetElement( "scaling");

	while( elem) {
		if( !( elem->HasAttribute( "collision"))) {
			gzwarn << "# lOS(): Skipping element without collision attribute"
				<< std::endl;

			elem = elem->GetNextElement( "scaling");
			continue;
		}

		auto name = elem->Get<std::string>( "collision");

		if( !( elem->HasAttribute( "scale"))) {
			gzerr << "# lOS(): Missing \"scale\" attribute in <scaling> "
				<< "element for \"" << name << "\"" << std::endl;

			elem = elem->GetNextElement( "scaling");
			continue;
		}

		auto scale = elem->Get< ignition::math::Vector3d>( "scale");
		_pScaleMapRet[ name] = scale;
		_nRet++;

		if( elem->HasAttribute( "pose")) {
			if( _bDebug) {
				gzmsg << "# lOS(): Optional \"pose\" attribute set in "
					<< "<scaling> element for \"" << name << "\"" << std::endl;
			}

			auto pose = elem->Get< ignition::math::Pose3d>( "pose");
			_pOffsetMapRet[ name] = pose;
		}

		elem = elem->GetNextElement( "scaling");
	}

	return _nRet;
}

// ########################################
unsigned int
TrajectoryActorExtendedPlugin::rescaleAllLinkCollision(
	const physics::LinkPtr &_link,
	const std::map< std::string, ignition::math::Vector3d> &_pScaleMap,
	const std::map< std::string, ignition::math::Pose3d> &_pOffsetMap,
	const bool &_bDebug)
{
	unsigned int _nRet = 0;

	// Process all the collisions in all the links
	for( const auto &collision : _link->GetCollisions()) {
		auto name = collision->GetName();

		if( _bDebug) {
			gzmsg << "# rALC(): collision \"" << name << "\"" << std::endl;
		}

		if( _pScaleMap.find(name) == _pScaleMap.end()) {
			if( _bDebug) {
				gzmsg << "# rALC(): no scaling for \"" << name << "\""
					<< std::endl;
			}
			continue;
		}

		auto boxShape = boost::dynamic_pointer_cast<
			gazebo::physics::BoxShape>( collision->GetShape());

		// Make sure we have a box shape.
		if( !boxShape) {
			gzerr << "# rALC(): not box shape for \"" << name << "\""
				<< std::endl;
			continue;
		}

		if( _bDebug) {
			gzmsg << "# rALC(): \"" << name << " (" << boxShape->TypeStr()
				<< ") size = " << boxShape->Size() << std::endl;
		}

		boxShape->SetSize( boxShape->Size() * _pScaleMap.at( name));
		_nRet++;

		if( _pOffsetMap.find( name) == _pOffsetMap.end()) {
			continue;
		}

		if( _bDebug) {
			gzmsg << "# rALC(): \"" << name
				<< " has optional custom pose offset." << std::endl;
		}

		collision->SetInitialRelativePose(
			_pOffsetMap.at( name) + collision->InitialRelativePose());
	}

	return _nRet;
}

// ########################################
unsigned int
TrajectoryActorExtendedPlugin::rescaleActorSkeleton(
	const std::map< std::string, ignition::math::Vector3d> &_pScaleMap,
	const std::map< std::string, ignition::math::Pose3d> &_pOffsetMap,
	const bool &_bDebug)
{
	unsigned int _nRet = 0;

	if( _bDebug) {
		gzmsg << "# rAS(): <actor> \"" << this->dataPtr->actor->GetName()
			<< "\" @ " << this->dataPtr->actor->WorldPose() << std::endl;
	}

	for( const auto &link : this->dataPtr->actor->GetLinks()) {
		// Init the links, which in turn enables collisions
		if( this->dataPtr->bUseSkeleton4Collision) {
			// WARNING:
			// calling link->Init() will cause TrajectoryActorExtendedPlugin
			// to fail to properly move Actor object (following trajectory) in
			// Gazebo.
			// This code fault tracked down by observer correct behaviour when:
			// - substituting TrajectoryActorExtendedPlugin.so with
			//   TrajectoryActorPlugin.so in test_single_green.world
			// - switching from new TrajectoryActorExtendedPlugin::OnUpdate()
			//   back to original TrajectoryActorPlugin::OnUpdate()
//			link->Init();
		}

		if( _bDebug) {
			gzmsg << "# rAS(): link = \"" << link->GetName() << "\""
				<< std::endl;
		}

		if( this->rescaleAllLinkCollision( link, _pScaleMap, _pOffsetMap,
				_bDebug) > 0)
		{
			if( this->dataPtr->bUseSkeleton4Collision) {
				link->Update();
			}
			_nRet++;
		}
	}

	return _nRet;
}

/////////////////////////////////////////////////
void
TrajectoryActorExtendedPlugin::Load( physics::ModelPtr _model,
	sdf::ElementPtr _sdf)
{
	this->dataPtr->actor =
		boost::dynamic_pointer_cast<physics::Actor>( _model);

	this->dataPtr->connections.push_back(
		event::Events::ConnectWorldUpdateBegin(
			std::bind( &TrajectoryActorExtendedPlugin::OnUpdate, this,
				std::placeholders::_1)));

	// Update frequency
	if( _sdf->HasElement( "update_frequency")) {
		this->dataPtr->updateFreq = _sdf->Get< double>( "update_frequency");
	}

	// Read in the velocity
	if( _sdf->HasElement( "velocity")) {
		this->dataPtr->velocity = _sdf->Get< double>( "velocity");
	}

	// Read in the target poses
	auto targetElem = _sdf->GetElement( "target");
	while( targetElem) {
		this->dataPtr->targets.push_back(
			targetElem->Get< ignition::math::Pose3d>());
		targetElem = targetElem->GetNextElement( "target");
	}

	// Read in the target mradius
	if( _sdf->HasElement("target_radius")) {
		this->dataPtr->targetRadius = _sdf->Get< double>( "target_radius");
	}

	// Read in the obstacle margin
	if( _sdf->HasElement("obstacle_margin")) {
		this->dataPtr->obstacleMargin = _sdf->Get< double>( "obstacle_margin");
	}

	// Read in the animation factor
	if( _sdf->HasElement("animation_factor")) {
		this->dataPtr->animationFactor =
			_sdf->Get< double>( "animation_factor");
	}

	const std::string _szMe( this->dataPtr->actor->GetName());
	
	this->loadDebugFlag( _sdf);
	const bool _bDebug( this->dataPtr->bDebugOnLoad);

	assert( this->dataPtr->actor->GetLink( "canonical"));

	this->dataPtr->model = this->dataPtr->actor->GetWorld()->ModelByName(
		_szMe + "_collision_model");

	assert( this->dataPtr->model != NULL);
	assert( this->dataPtr->model->GetLink( "canonical"));

	this->dataPtr->mainLink =
		this->dataPtr->actor->GetChildLink( _szMe + "_pose");

	assert( this->dataPtr->mainLink != NULL);

	if( _bDebug) {
		gzmsg << "# onL(): actor canonical = \""
			<< this->dataPtr->actor->GetLink( "canonical")->GetName() << "\""
			<< std::endl;
		gzmsg << "# onL(): collision model canonical = \""
			<< this->dataPtr->model->GetLink( "canonical")->GetName() << "\""
			<< std::endl;
		gzmsg << "# onL(): actor mainLink = \""
			<< this->dataPtr->mainLink->GetName() << "\"" << std::endl;
	}

	if( _bDebug) {
		gzmsg << "# onL(): IGN_PI_2 = " << IGN_PI_2 << " IGN_DTOR(10) = "
			<< IGN_DTOR(10) << std::endl;

		gzmsg << "WTF-1a \"" << _szMe << "\"" << std::endl;
		gzmsg << "WTF-1b \"" << _szMe << "\" on_load="
			<< this->dataPtr->bDebugOnLoad << ", on_update="
			<< this->dataPtr->bDebugOnUpdate << ", on_adhoc="
			<< this->dataPtr->bDebugOnAdHoc << std::endl;

		gzmsg << "WTF-1c \"" << _szMe << "\" HasType( LINK) = "
			<< (this->dataPtr->actor->HasType(
				physics::Model::EntityType::LINK) ? "Y" : "N")
			<< " HasType( MODEL) ? "
			<< (this->dataPtr->actor->HasType(
				physics::Model::EntityType::MODEL) ? "Y" : "N")
			<< " IsCanonicalLink() ? "
			<< (this->dataPtr->actor->IsCanonicalLink() ? "Y" : "N")
			<< std::endl;

		gzmsg << "WTF-1d \"" << _szMe << "\" GetChildCount() = "
			<< this->dataPtr->actor->GetChildCount() << std::endl;

		for( unsigned int j = 0; j < this->dataPtr->actor->GetChildCount();
			++j)
		{
			gzmsg << "[" << j << "]: \""
				<< this->dataPtr->actor->GetChild( j)->GetName() << "\""
				<< std::endl;
		}
	}

	// Read in the obstacles
	if( _sdf->HasElement( "obstacle")) {
		auto world = this->dataPtr->actor->GetWorld();

		auto obstacleElem = _sdf->GetElement( "obstacle");
		while( obstacleElem) {
			auto name = obstacleElem->Get< std::string>();
			this->dataPtr->obstacles.push_back( name);

			this->checkForModelChildLink( world, name, "door", true);

			obstacleElem = obstacleElem->GetNextElement( "obstacle");
		}
	}

	// Read in the animation name
	std::string animation{ "animation"};
	if( _sdf->HasElement( "animation")) {
		animation = _sdf->Get< std::string>( "animation");
	}

	auto skelAnims = this->dataPtr->actor->SkeletonAnimations();
	if( skelAnims.find( animation) == skelAnims.end()) {
		gzerr << "Skeleton animation [" << animation << "] not found in Actor."
			<< std::endl;
	}
	else {
		// Set custom trajectory
		gazebo::physics::TrajectoryInfoPtr trajectoryInfo(
			new physics::TrajectoryInfo());
		trajectoryInfo->type = animation;
		trajectoryInfo->duration = 1.0;

		this->dataPtr->actor->SetCustomTrajectory( trajectoryInfo);
	}


	// Map of collision scaling factors
	std::map<std::string, ignition::math::Vector3d> scaling;
	std::map<std::string, ignition::math::Pose3d> offsets;

	if( _bDebug) {
		gzmsg << "WTF-2a \"" << _szMe << "\"" << std::endl;
	}
	if( this->loadOptionalScaling( scaling, offsets, _sdf,
			this->dataPtr->bDebugOnLoad) > 0)
	{
		if( _bDebug) {
			gzmsg << "WTF-2b \"" << _szMe << "\"" << std::endl;
		}
		this->dataPtr->bUseSkeleton4Collision = true;
		if( _bDebug) {
			gzmsg << "WTF-2c \"" << _szMe << "\"" << std::endl;
		}
		this->rescaleActorSkeleton( scaling, offsets,
			this->dataPtr->bDebugOnLoad);
		if( _bDebug) {
			gzmsg << "WTF-2d \"" << _szMe << "\"" << std::endl;
		}
	}


	const std::string _szTopic4WayPt = _sdf->Get<std::string>( "topic",
		"/gazebo/actor/way_pt").first;

	this->dataPtr->hdrMsg.seq = 0;
	this->dataPtr->hdrMsg.frame_id = _szMe;

	if( !( _szTopic4WayPt.empty())) {
		this->dataPtr->rosnode_ = new ros::NodeHandle();

		// Create the publisher of header messages
		this->dataPtr->pub_ = this->dataPtr->rosnode_->advertise<
			std_msgs::Header>( _szTopic4WayPt, 1);
	}

	const std::string _szTopic4LookAhead = _sdf->Get<std::string>(
		"topic_look_ahead", "/gazebo/actor/look_ahead").first;

	this->dataPtr->pub4LookAhead = this->dataPtr->rosnode_->advertise<
		visualization_msgs::Marker>( _szTopic4LookAhead + "/" + _szMe, 1);
}

/////////////////////////////////////////////////
void TrajectoryActorExtendedPlugin::Reset()
{
  this->dataPtr->currentTarget = 0;
  this->dataPtr->cornerAnimation = nullptr;
  this->dataPtr->lastUpdate = common::Time::Zero;
}

// ########################################
bool
TrajectoryActorExtendedPlugin::testForCollisionNonActor(
	const std::string &_szMe, const physics::ModelPtr &_model,
	const std::string &_szNameModel, const std::string &_szNameChild,
	const BBOX_TYPE &_bbReal,
	bool &_bLastCollideNonActor, const bool &_bDebug) const
{
	if( _model->GetName() != _szNameModel) {
		return false;
	}

	auto _child = _model->GetChild( _szNameChild);

	if( !_child) {
		if( _bDebug) {
			gzerr << "# t4CNonA(): Model \"" << _model->GetName() << "\" does "
				<< "not have child \"" << _szNameChild << "\"" << std::endl;
		}
		return false;
	}

	assert( _child->HasType( physics::Model::EntityType::LINK));
	auto _cLink = boost::dynamic_pointer_cast< physics::Link>( _child);

	if( _bDebug) {
		gzmsg << "# t4CNonA(): " << _szNameChild << " BBox = "
			<< _cLink->BoundingBox() << std::endl;
	}

	if( _cLink->BoundingBox().Intersects( _bbReal)) {
		if( !_bLastCollideNonActor) {
			_bLastCollideNonActor = true;

			if( _bDebug || this->dataPtr->bDebugOnAdHoc) {
//				gzmsg << "# t4CNonA(): " << _szMe << ": INTERSECTS !!!"
//					<< std::endl;
				gzwarn << std::endl << "# t4CNonA(): Collision (A-X): \""
					<< _szMe << "\" v.s. \"" << _model->GetName() << "\""
					<< std::endl;
				gzwarn << "# t4CNonA(): bbox (A v.s X) = " << _bbReal << " / "
					<< _cLink->BoundingBox() << std::endl;
			}
		}
		return true;
	}

	if( _bDebug) {
		gzmsg << "# t4CNonA(): " << _szMe << ": No Intersection ..."
			<< std::endl;
	}

	return false;
}

// ########################################
int
TrajectoryActorExtendedPlugin::testForCollisionActor( const std::string &_szMe,
	const physics::ModelPtr &_model, const physics::WorldPtr &_world,
	const BBOX_TYPE &_bbReal,
	bool &_bLastCollideActor,  const bool &_bDebug) const
{
	auto _cActor = _world->ModelByName( _model->GetName() + "_collision_model");

	if( !_cActor) {
		if( _bDebug) {
			gzmsg << "# t4CA(): No _collision_model => static human? \""
				<< _szMe << "\"" << std::endl;
		}
		return -1;
	}

	// Don't do it this way, or 2 actors will cause each other to stop
	// permanently once their bounding boxes intersect.
	// Fall through and do it the naive original way instead.
	if( false && _cActor->BoundingBox().Intersects( _bbReal)) {
		if( _bDebug) {
			gzwarn << "# t4CA(): Collision (A-A) " << _szMe << " v.s. "
						<< _model->GetName() << std::endl;
		}
		return 1;
	}

	// Fall through to simplistic original check below.
	return -1;
}

// ########################################
int
TrajectoryActorExtendedPlugin::testModelForCollision( const std::string &_szMe,
	const unsigned int &_i, const physics::ModelPtr &_model,
	const physics::WorldPtr &_world,
	const BBOX_TYPE &_bbReal,
	bool &_bLastCollideNonActor, bool &_bLastCollideActor,
	const bool &_bDebug) const
{
	int _nRet = -1;

	const bool _bIsActor( _model->HasType( physics::Model::EntityType::ACTOR));

	if( _bDebug) {
		const std::string _szTmp( _bIsActor ? " *" : "");
		gzwarn << "# tM4C(): [" << _i << "]: \"" << _model->GetName() << "\""
			<< _szTmp << std::endl;

		if( !_bIsActor) {
			gzmsg << "# tM4C(): [" << _i << "]: GetChildCount() = "
				<< _model->GetChildCount() << std::endl;

			for( unsigned int j = 0; j < _model->GetChildCount(); ++j) {
				gzmsg << "[" << _i << "." << j << "]: \""
					<< _model->GetChild( j)->GetName() << "\"" << std::endl;
			}
		}
	}

	if( !_bIsActor) {
		_nRet = this->testForCollisionNonActor( _szMe, _model, "PublicCafe",
			"door", _bbReal, _bLastCollideNonActor, _bDebug) ? 2 : 0;

		if( _bDebug) {
			gzmsg << "# tM4C(): [" << _i << "]: \"" << _szMe << "\" v.s. \""
				<< _model->GetName() << "\" _nRet = " << _nRet
				<< " (non-actor)" << std::endl;
		}
	}
	else {
		_nRet = this->testForCollisionActor( _szMe, _model, _world, _bbReal,
			_bLastCollideActor, _bDebug);
	}

	return _nRet;
}

/////////////////////////////////////////////////
bool TrajectoryActorExtendedPlugin::ObstacleOnTheWay() const
{
  auto actorWorld = ignition::math::Matrix4d(this->dataPtr->actor->WorldPose());
  auto world = this->dataPtr->actor->GetWorld();

  const std::string szMe( this->dataPtr->actor->GetName());
  auto bbMe = this->dataPtr->actor->BoundingBox();

  const bool _bFirstDebug =
      this->dataPtr->bDebugOnUpdate && this->dataPtr->bFirstIter;

  this->dataPtr->bFirstIter = false;

  auto plugin = this->dataPtr->actor->GetByName( "attach_model");

  if( _bFirstDebug) {
    gzmsg << "# OOtW(): <actor name=\"" << szMe << "\">"
      << ":GetPluginCount() = " << this->dataPtr->actor->GetPluginCount()
      // << " NestedModels() " << "[" << nested.size() << "]"
      << ", GetChildCount() = " << this->dataPtr->actor->GetChildCount()
      << ", GetByName( \"attach_model\") ? " << (plugin ? "Y" : "N")
      << std::endl;
    gzmsg << "# OOtW(): BB Min(): " << bbMe.Min() << std::endl;
    gzmsg << "# OOtW(): BB Max(): " << bbMe.Max() << std::endl;
  }

  if( _bFirstDebug) {
    for( auto i = 0; i < this->dataPtr->actor->GetChildCount(); ++i) {
      gzmsg << "#   [" << i << "] \"" << this->dataPtr->actor->GetChild(
          i)->GetName() << "\" isA " << this->dataPtr->actor->GetChild(
          i)->GetType() << std::endl;
    }
  }

  BBOX_TYPE bbReal;

  auto cModel = world->ModelByName( szMe + "_collision_model");

  if( cModel) {
    bbReal = cModel->BoundingBox();
    if( _bFirstDebug) {
      gzmsg << "# OOtW(): got associated model \"" << cModel->GetName() << "\""
        << " B-box = " << cModel->BoundingBox()
        << " B-copy = " << bbReal
        << std::endl;
    }
  }
  else {
    // assert( false);
  }

  bool _bLastCollideStatic = this->dataPtr->pbLastCollideStatic[ szMe];
  bool _bLastCollideActor = this->dataPtr->pbLastCollideActor[ szMe];
  int _nRet = 0;

  // Iterate over all models in the world
  for (unsigned int i = 0; i < world->ModelCount(); ++i)
  {
    // Skip if it's not an obstacle
    // Fixme: automatically adding all actors to obstacles
    auto model = world->ModelByIndex(i);
    auto act = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
    if (!act &&
        std::find(this->dataPtr->obstacles.begin(),
                  this->dataPtr->obstacles.end(), model->GetName()) ==
                  this->dataPtr->obstacles.end())
    {
      if( _bFirstDebug) {
        gzmsg << "# OOtW(): [" << i << "]: Skip \"" << model->GetName() << "\""
          << std::endl;
      }
      continue;
    }
    if (act && act == this->dataPtr->actor) {
      if( _bFirstDebug) {
        gzwarn << "# OOtW(): [" << i << "]: Self \"" << model->GetName()
          << "\"" << std::endl;
      }
      continue;
    }

    if( (_nRet = this->testModelForCollision( szMe, i, model, world, bbReal,
          _bLastCollideStatic, _bLastCollideActor, _bFirstDebug)) > 0)
    {
        break;
    }

    auto modelWorld = ignition::math::Matrix4d(model->WorldPose());

    // Model in actor's frame
    auto modelActor = actorWorld.Inverse() * modelWorld;

    auto modelPos = ignition::math::Vector3d(
        modelActor(0, 3),
        modelActor(1, 3),
        modelActor(2, 3));

    const double _fScaledMargin = this->dataPtr->obstacleMargin *
        std::max( this->dataPtr->velocity, 1.0);

    if( this->dataPtr->bDebugOnAdHoc
        // && (szMe == "human_80283")
        && (model->GetName() == "husky"))
    {
    if( modelPos.Length() < _fScaledMargin) {
      if( this->dataPtr->bDebugOnAdHoc && !_bLastCollideActor) {
        gzmsg << "# OOtW(): v.s. husky modelPos.Length() v.s. _fScaledMargin: "
            << modelPos.Length() << " / " << _fScaledMargin << std::endl;
        gzmsg << "# OOtW(): v.s. husky abs(modelActor(0, 3) v.s. "
            << "obstacleMargin * 0.5 = " << std::abs( modelActor(0, 3))
            << " / " << (this->dataPtr->obstacleMargin * 0.67) << std::endl;
        gzmsg << "# OOtW(): v.s. husky modelActor(2, 3) = "
            << modelActor(2, 3) << std::endl;
      }
    }
    else {
      if( _bLastCollideActor) {
        gzmsg << "# OOtW(): XXXX husky modelPos.Length() v.s. _fScaledMargin: "
            << modelPos.Length() << " / " << _fScaledMargin << std::endl;
        gzmsg << "# OOtW(): XXXX husky abs(modelActor(0, 3) v.s. "
            << "obstacleMargin * 0.5 = " << std::abs( modelActor(0, 3))
            << " / " << (this->dataPtr->obstacleMargin * 0.67) << std::endl;
        gzmsg << "# OOtW(): XXXX husky modelActor(2, 3) = "
            << modelActor(2, 3) << std::endl;
      }
    }
    }

    // Check not only if near, but also if in front of the actor
    if( (modelPos.Length() < _fScaledMargin) &&
        (std::abs(modelActor(0, 3)) < this->dataPtr->obstacleMargin * 0.67) &&
        (modelActor(2, 3) > 0))
    {
      if( !_bLastCollideActor) {
        _bLastCollideActor = true;

        if( _bFirstDebug || this->dataPtr->bDebugOnAdHoc) {
          gzwarn << std::endl << "# OOtW(): Collision (A-A): \"" << szMe
            << "\" v.s. \"" << model->GetName()
            << "\", modelPos.Length() v.s. obstacleMargin = "
            << modelPos.Length() << " / " << this->dataPtr->obstacleMargin
            << std::endl;
          gzwarn << "# OOtW(): modelActor = " << modelActor << " (0,3) = "
            << modelActor(0,3) << " (2,3) = " << modelActor(2,3) << std::endl;
        }
      }
      _nRet = 3;
      break;

      return true;
    }

    // TODO: Improve obstacle avoidance. Some ideas: check contacts, ray-query
    // the path forward, check against bounding box of each collision shape...

    // Note: Used to check against model bounding box, but that was unreliable
    // for URDF and actors (our 2 use cases :))
  }

  if( (_nRet == 2)) {
    this->dataPtr->pbLastCollideStatic[ szMe] = true;
  }
  else if( (_nRet == 1) || (_nRet == 3)) {
    this->dataPtr->pbLastCollideActor[ szMe] = true;
  }
  else {
    this->dataPtr->pbLastCollideStatic[ szMe] = false;
    this->dataPtr->pbLastCollideActor[ szMe] = false;
  }

  return (_nRet > 0);
  return false;
}

/////////////////////////////////////////////////
bool TrajectoryActorExtendedPlugin::UpdateTarget()
{
//  const bool _bDebug( true);
  // Current actor position
  auto actorPos = this->dataPtr->actor->WorldPose().Pos();

  // Current target
  auto target = this->dataPtr->targets[this->dataPtr->currentTarget].Pos();

  // 2D distance to target
  auto posDiff = target - actorPos;
  posDiff.Z(0);

  double distance = posDiff.Length();

  // Still far from target?
  if (distance > this->dataPtr->targetRadius)
//    if( _bDebug) {
//      this->dataPtr->nSameLastTarget++;
//    }
    return false;

//  if( _bDebug) {
//    gzmsg << "# UT(): close to target [" << this->dataPtr->currentTarget
//      << "] after " << this->dataPtr->nSameLastTarget << " cycle(s)"
//      << std::endl;
//    this->dataPtr->nSameLastTarget = 0;
//  }

  // Move on to next target
  this->dataPtr->currentTarget++;
  if (this->dataPtr->currentTarget > this->dataPtr->targets.size() - 1)
    this->dataPtr->currentTarget = 0;

  if( this->dataPtr->pub_) {
    this->dataPtr->hdrMsg.seq = this->dataPtr->currentTarget;
    this->dataPtr->pub_.publish( this->dataPtr->hdrMsg);
  }

  return true;
}


// ############################################################
void
TrajectoryActorExtendedPlugin::genLookAheadStraightLine(
	const ignition::math::Pose3d &_tgt, const ignition::math::Pose3d &_current,
	const ignition::math::Vector3d &_dir, const double &_dWindow)
{
	const double _dMax = _current.Pos().Distance( _tgt.Pos());
	const int _nInt = std::ceil( _dMax / this->dataPtr->velocity);

	ignition::math::Pose3d _intraPt( _current);

	visualization_msgs::Marker mFuturePose;

	for( int i = 0; i < _nInt; ++i) {
		geometry_msgs::Point _pt;
		_pt.x = _intraPt.Pos().X();
		_pt.y = _intraPt.Pos().Y();
		_pt.z = _intraPt.Pos().Z();
		_intraPt.Pos() += _dir;
		mFuturePose.points.push_back( _pt);
	}

//	mFuturePose.lifetime = ros::Duration( 1);
	mFuturePose.lifetime = ros::Duration( 1 / this->dataPtr->velocity);
	mFuturePose.text = this->dataPtr->hdrMsg.frame_id;
//	mFuturePose.type = visualization_msgs::Marker::POINTS;
	mFuturePose.type = visualization_msgs::Marker::SPHERE_LIST;
	mFuturePose.action = visualization_msgs::Marker::ADD;

	mFuturePose.ns = "prediction_fake";
	mFuturePose.id = 999;

	mFuturePose.scale.x = mFuturePose.scale.y = 0.2;
	mFuturePose.scale.z = 1.0;

	mFuturePose.color.r = 1.0;
	mFuturePose.color.g = 0.0;
	mFuturePose.color.b = 1.0;
	mFuturePose.color.a = 1.0;

	mFuturePose.header.frame_id = "world";
	mFuturePose.header.stamp = ros::Time::now();

	this->dataPtr->pub4LookAhead.publish( mFuturePose);
}

/////////////////////////////////////////////////
// NOTE: set pre-compiler condition to 0 to switch OnUpdate() back to original
// version implemented as TrajectoryActorPlugin::OnUpdate()
#if 1
void
TrajectoryActorExtendedPlugin::OnUpdate( const common::UpdateInfo &_info)
{
	// Time delta
	double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

	const bool _bDebug( this->dataPtr->bDebugOnUpdate);

	if( dt < 1 / this->dataPtr->updateFreq) {
		if( _bDebug) {
			gzmsg << "# onU(): same time interval ["
				<< this->dataPtr->nSameTimeInt << "] dt = " << dt
				<< " < (1 / " << this->dataPtr->updateFreq << ")" << std::endl;
		}
		this->dataPtr->nSameTimeInt++;
		return;
	}

	if( _bDebug) {
		gzmsg << "# onU(): skipped " << this->dataPtr->nSameTimeInt
			<< " same time interval(s)" << std::endl;
	}

	if( this->dataPtr->nIntraCycleMax < this->dataPtr->nSameTimeInt) {
		this->dataPtr->nIntraCycleMax = this->dataPtr->nSameTimeInt;
	}

	this->dataPtr->nSameTimeInt = 0;
	this->dataPtr->lastUpdate = _info.simTime;

	// Don't move if there's an obstacle on the way
	if( this->ObstacleOnTheWay()) {
		return;
	}

	// Target
	const bool _bClose = this->UpdateTarget();
	if( _bClose) {
		if( _bDebug) {
			gzmsg << "# UT(): close to target ["
				<< this->dataPtr->currentTarget << "] after "
				<< this->dataPtr->nSameLastTarget << " cycle(s)" << std::endl;
		}
		this->dataPtr->nSameLastTarget = 0;
	}
	else {
		this->dataPtr->nSameLastTarget++;
	}

	// Current pose - actor is oriented Y-up and Z-front
	ignition::math::Pose3d actorPose( this->dataPtr->actor->WorldPose());

	// Current target
	const ignition::math::Pose3d targetPose(
		this->dataPtr->targets[ this->dataPtr->currentTarget]);

	// Direction to target
	const ignition::math::Vector3d dir(
//		targetPose.Pos() - actorPose.Pos()
		(targetPose.Pos() - actorPose.Pos()).Normalize()
	);

	// TODO: generalize for actors facing other directions
	const double currentYaw = actorPose.Rot().Yaw();

	// Difference to target
	ignition::math::Angle yawDiff( (true || _bClose) ? (
//		atan2( dir.Y(), dir.X()) - currentYaw
		atan2( dir.Y(), dir.X()) + IGN_PI_2 - currentYaw
	) : ignition::math::Angle::Zero
	);

	if( _bDebug) {
		gzmsg << "# OU(): before Normalize(): yawDiff / yawDiff.Radian() = "
			<< yawDiff << " / " << yawDiff.Radian() << std::endl;
	}

	yawDiff.Normalize();

	if( _bDebug) {
		gzmsg << "# OU(): Current -> target [" << this->dataPtr->currentTarget
			<< "] = " << actorPose << " / " << targetPose << std::endl;
		gzmsg << "# OU(): yawDiff.Radian() = " << yawDiff.Radian()
			<< " dirY/X/Z = " << dir.Y() << ", " << dir.X() << ", " << dir.Z()
			<< " (" << atan2( dir.Y(), dir.X()) << ")" << " v.s. "
			<< currentYaw << std::endl;
	}

	double _dWin = 1 / this->dataPtr->velocity;

	// Rotate if needed
	if( std::abs( yawDiff.Radian()) > IGN_DTOR(10)) {
		if( _bDebug) {
			gzmsg << "# OU(): CCC " << yawDiff.Radian() << " / "
				<< IGN_DTOR(10) << std::endl;
		}

		// Not rotating yet
		if( !this->dataPtr->cornerAnimation) {
			// Previous target (we assume we just reached it)
			int previousTarget = this->dataPtr->currentTarget - 1;
			if( previousTarget < 0) {
				previousTarget = this->dataPtr->targets.size() - 1;
			}

			auto prevTargetPos = this->dataPtr->targets[ previousTarget].Pos();

			// Direction from previous target to current target
			auto prevDir = (targetPose.Pos() - prevTargetPos).Normalize() *
				this->dataPtr->targetRadius;

			// Curve end point
			auto endPt = prevTargetPos + prevDir;

			if( _bDebug) {
				gzmsg << "# onU() prevDir = " << prevDir << ", endPt = "
					<< endPt << std::endl;
			}

			// Total time to finish the curve, we try to keep about the same,
			// speed it will be a bit slower because we're doing an arc, not
			// a straight line
			auto curveDist = (endPt - actorPose.Pos()).Length();
			auto curveTime = curveDist / this->dataPtr->velocity;

			if( _bDebug) {
				gzmsg << "# onU() curveDist = " << curveDist
					<< ", curveTime = " << curveTime << std::endl;
			}

			// Use pose animation for spline
			this->dataPtr->cornerAnimation = new common::PoseAnimation(
				"anim", curveTime, false);

			// Start from actor's current pose
			auto start = this->dataPtr->cornerAnimation->CreateKeyFrame( 0.0);
			start->Translation( actorPose.Pos());
			start->Rotation( actorPose.Rot());

			// End of curve
			auto endYaw = atan2( prevDir.Y(), prevDir.X()) + IGN_PI_2;
//			auto endYaw = atan2( dir.Y(), dir.X());
			auto end = this->dataPtr->cornerAnimation->CreateKeyFrame(
				curveTime);

			end->Translation( endPt);
			end->Rotation( ignition::math::Quaterniond( IGN_PI_2, 0, endYaw));

			this->dataPtr->firstCornerUpdate = _info.simTime;

			if( _bDebug) {
				gzmsg << "# OU(): AAA-0 " << _info.simTime << " v.s. "
					<< this->dataPtr->firstCornerUpdate << std::endl;
			}
		}

		// Get point in curve
		if( _bDebug) {
			gzmsg << "# OU(): EEE simTime v.s. firstCornerUpdate = "
				<< _info.simTime << " / " << this->dataPtr->firstCornerUpdate
				<< std::endl;
		}
		auto cornerDt = (_info.simTime -
			this->dataPtr->firstCornerUpdate).Double();

		common::PoseKeyFrame pose( cornerDt);

		this->dataPtr->cornerAnimation->SetTime( cornerDt);
		this->dataPtr->cornerAnimation->GetInterpolatedKeyFrame( pose);

		actorPose.Pos() = pose.Translation();
		actorPose.Rot() = ignition::math::Quaterniond(
			IGN_PI_2, 0, pose.Rotation().Yaw());

		if( _bDebug) {
			gzmsg << "# OU(): AAA " << cornerDt << " / " << actorPose.Pos()
				<< " / " << actorPose.Rot() << " pose.Rotation().Yaw() = "
				<< pose.Rotation().Yaw() << std::endl;
		}
	}
	else {
		_dWin = this->dataPtr->dMaxLookAhead;

		this->dataPtr->cornerAnimation = nullptr;

		actorPose.Pos() += dir * this->dataPtr->velocity * dt;

		// TODO: remove hardcoded roll
		actorPose.Rot() = ignition::math::Quaterniond(
			IGN_PI_2, 0, currentYaw + yawDiff.Radian());

		if( _bDebug) {
			gzmsg << "# OU(): BBB currentYaw v.s. yawDiff = " << currentYaw
				<< " / " << yawDiff.Radian() << std::endl;
		}
	}

	this->genLookAheadStraightLine( targetPose, actorPose, dir, _dWin);

	// Distance traveled is used to coordinate motion with the walking
	// animation
	double distanceTraveled =
//		(actorPose.Pos() - this->dataPtr->actor->WorldPose().Pos()).Length();
		actorPose.Pos().Distance( this->dataPtr->actor->WorldPose().Pos());

	if( _bDebug) {
		gzmsg << "# OU(): actorPose.Pos() v.s. actor->WorldPose().Pos() = "
			<< actorPose.Pos() << " / "
			<< this->dataPtr->actor->WorldPose().Pos() << std::endl;
		gzmsg << "# OU(): actorPose.Rot() v.s. actor->WorldPose().Rot() = "
			<< actorPose.Rot() << " / "
			<< this->dataPtr->actor->WorldPose().Rot() << std::endl;
		gzmsg << "# OU(): distanceTravelled = "
			<< distanceTraveled << std::endl;
	}

	// Update actor
	this->dataPtr->actor->SetWorldPose( actorPose,
		false && this->dataPtr->bUseSkeleton4Collision, false);

	const double lx = cos( actorPose.Rot().Yaw()) * this->dataPtr->velocity;
	const double ly = sin( actorPose.Rot().Yaw()) * this->dataPtr->velocity;
	const double az = yawDiff.Radian() / dt;
//	const double az = 0;
//	const double az =
//		(yawDiff.Radian() / dt) / this->dataPtr->nIntraCycleMax;

	if( _bDebug) {
		gzmsg << "# Yaw() v.s. linear.x/y = " << actorPose.Rot().Yaw() << " / "
			<< "[ " << lx << ", " << ly << " ] yawDiff.Radian() / dt = "
			<< yawDiff.Radian() << " / " << dt << " (/ "
			<< this->dataPtr->nIntraCycleMax << ") = " << az
			<< (_bClose ? " (*)" : "") << std::endl;
	}

	// Won't work on this->dataPtr->actor - because Actor::HasType() returns
	// false for Link, true for Model, but SetWorldPose() only calls underlying
	// SetLinearVel() & SetAngularVel() for Link types.
//	this->dataPtr->model->SetWorldTwist(
//		ignition::math::Vector3d( lx, ly, 0.0),
//		ignition::math::Vector3d( 0.0, 0.0, az));

//	this->dataPtr->mainLink->SetLinearVel(
//		ignition::math::Vector3d( lx, ly, 0.0));
//	this->dataPtr->mainLink->SetAngularVel(
//		ignition::math::Vector3d( 0.0, 0.0, az));

	if( _bDebug) {
		gzmsg << "# onU(): actor::WorldLinearVel() = "
			<< this->dataPtr->actor->WorldLinearVel() << std::endl;
		gzmsg << "# onU(): SetScriptTime( "
			<< this->dataPtr->actor->ScriptTime() << " + (" << distanceTraveled
			<< " * " << this->dataPtr->animationFactor << ") = "
			<< (this->dataPtr->actor->ScriptTime() +
				(distanceTraveled * this->dataPtr->animationFactor))
			<< std::endl;
	}

	this->dataPtr->actor->SetScriptTime( this->dataPtr->actor->ScriptTime() +
		(distanceTraveled * this->dataPtr->animationFactor));
}
#else
void TrajectoryActorExtendedPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  if (dt < 1/this->dataPtr->updateFreq)
    return;

  this->dataPtr->lastUpdate = _info.simTime;

  // Don't move if there's an obstacle on the way
  if (this->ObstacleOnTheWay())
    return;

  // Target
  this->UpdateTarget();

  // Current pose - actor is oriented Y-up and Z-front
  auto actorPose = this->dataPtr->actor->WorldPose();

  // Current target
  auto targetPose = this->dataPtr->targets[this->dataPtr->currentTarget];

  // Direction to target
  auto dir = (targetPose.Pos() - actorPose.Pos()).Normalize();

  // TODO: generalize for actors facing other directions
  auto currentYaw = actorPose.Rot().Yaw();

  // Difference to target
  ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
  yawDiff.Normalize();

  // Rotate if needed
  if (std::abs(yawDiff.Radian()) > IGN_DTOR(10))
  {
    // Not rotating yet
    if (!this->dataPtr->cornerAnimation)
    {
      // Previous target (we assume we just reached it)
      int previousTarget = this->dataPtr->currentTarget - 1;
      if (previousTarget < 0)
        previousTarget = this->dataPtr->targets.size() - 1;

      auto prevTargetPos = this->dataPtr->targets[previousTarget].Pos();

      // Direction from previous target to current target
      auto prevDir = (targetPose.Pos() - prevTargetPos).Normalize() *
          this->dataPtr->targetRadius;

      // Curve end point
      auto endPt = prevTargetPos + prevDir;

      // Total time to finish the curve, we try to keep about the same speed,
      // it will be a bit slower because we're doing an arch, not a straight
      // line
      auto curveDist = (endPt - actorPose.Pos()).Length();
      auto curveTime = curveDist / this->dataPtr->velocity;

      // Use pose animation for spline
      this->dataPtr->cornerAnimation = new common::PoseAnimation("anim", curveTime, false);

      // Start from actor's current pose
      auto start = this->dataPtr->cornerAnimation->CreateKeyFrame(0.0);
      start->Translation(actorPose.Pos());
      start->Rotation(actorPose.Rot());

      // End of curve
      auto endYaw = atan2(prevDir.Y(), prevDir.X()) + IGN_PI_2;
      auto end = this->dataPtr->cornerAnimation->CreateKeyFrame(curveTime);
      end->Translation(endPt);
      end->Rotation(ignition::math::Quaterniond(IGN_PI_2, 0, endYaw));

      this->dataPtr->firstCornerUpdate = _info.simTime;
    }

    // Get point in curve
    auto cornerDt = (_info.simTime - this->dataPtr->firstCornerUpdate).Double();
    common::PoseKeyFrame pose(cornerDt);
    this->dataPtr->cornerAnimation->SetTime(cornerDt);
    this->dataPtr->cornerAnimation->GetInterpolatedKeyFrame(pose);

    actorPose.Pos() = pose.Translation();
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, pose.Rotation().Yaw());
  }
  else
  {
    this->dataPtr->cornerAnimation = nullptr;

    actorPose.Pos() += dir * this->dataPtr->velocity * dt;

    // TODO: remove hardcoded roll
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian());
  }

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->dataPtr->actor->WorldPose().Pos()).Length();

  // Update actor
  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() +
    (distanceTraveled * this->dataPtr->animationFactor));
}
#endif
