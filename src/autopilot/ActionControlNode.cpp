#include "ActionControlNode.h"

#include "KI/KIAutoInit.h"
#include "KI/KIFlyTo.h"
#include "KI/KILand.h"
#include "KI/KIProcedure.h"

using namespace tum_ardrone;

bool ActionControlNode::setReference(SetReference::Request& req, SetReference::Response& res)
{
	parameter_referenceZero = DronePosition(TooN::makeVector(req.x, req.y, req.z),
		req.heading);	
	res.status = true;
	return true;
}

bool ActionControlNode::setMaxControl(SetMaxControl::Request& req, SetMaxControl::Response& res)
{
	parameter_MaxControl = req.speed;
	res.status = true;
	return true;
}

bool ActionControlNode::setInitialReachDist(SetInitialReachDistance::Request& req, SetInitialReachDistance::Response& res)
{
	parameter_InitialReachDist = req.distance;
	res.status = true;
	return true;
}

bool ActionControlNode::setStayWithinDist(SetStayWithinDist::Request& req, SetStayWithinDist::Response& res) {
	parameter_StayWithinDist = req.distance;
	res.status = true;
	return true;
}

bool ActionControlNode::setStayTime(SetStayTime::Request& req, SetStayTime::Response& res) {
	parameter_StayTime = req.duration;
	res.status = true;
	return true;
}

bool ActionControlNode::start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	this->startControl();
	return true;
}

bool ActionControlNode::stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	this->stopControl();
	return true;
}

bool ActionControlNode::clear(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	this->clearCommands();
	return true;
}

bool ActionControlNode::hover(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	this->sendControlToDrone(hoverCommand);
	return true;
}

bool ActionControlNode::lockScaleFP(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	this->publishCommand("p lockScaleFP");
	return true;
}

void ActionControlNode::autoInit() {
	currentAction = "autoInit";
	ROS_INFO("calling action autoInit");
	AutoInitGoalConstPtr g = autoInit_->acceptNewGoal();
	currentKI = new KIAutoInit(true, g->moveTime, g->waitTime, g->riseTime, g->initSpeed, true);
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::autoTakeover() {
	currentAction = "autoTakeover";
	ROS_INFO("calling action autoTakeover");
	AutoInitGoalConstPtr g = autoTakeover_->acceptNewGoal();
	currentKI = new KIAutoInit(true, g->moveTime, g->waitTime, g->riseTime, g->initSpeed, false);
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::land() {
	currentAction = "land";
	ROS_INFO("calling action land");
	land_->acceptNewGoal();
	currentKI = new KILand();
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::takeoff() {
	currentAction = "takeoff";
	ROS_INFO("calling action takeoff");
	takeoff_->acceptNewGoal();
	currentKI = new KIAutoInit(false);
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::goTo() {
	currentAction = "goto";
	ROS_INFO("calling action goto");
	MoveGoalConstPtr g = goto_->acceptNewGoal();
	currentKI = new KIFlyTo(
		DronePosition(TooN::makeVector(g->x, g->y, g->z) + parameter_referenceZero.pos,
			g->heading + parameter_referenceZero.yaw),
		parameter_StayTime, parameter_MaxControl, parameter_InitialReachDist, parameter_StayWithinDist);
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::moveBy() {
	currentAction = "moveBy";
	ROS_INFO("calling action moveBy");
	MoveGoalConstPtr g = moveBy_->acceptNewGoal();
	currentKI = new KIFlyTo(
		DronePosition(TooN::makeVector(g->x, g->y, g->z) + controller.getCurrentTarget().pos,
			g->heading + controller.getCurrentTarget().yaw),
		parameter_StayTime, parameter_MaxControl, parameter_InitialReachDist, parameter_StayWithinDist);
	currentKI->setPointers(this, &controller);
}

void ActionControlNode::moveByRel() {
	currentAction = "moveByRel";
	ROS_INFO("calling action moveByRel");
	MoveGoalConstPtr g = moveByRel_->acceptNewGoal();
	currentKI = new KIFlyTo(
		DronePosition(TooN::makeVector(g->x + state->x, g->y + state->y, g->z + state->z),
			g->heading + state->yaw),
		parameter_StayTime, parameter_MaxControl, parameter_InitialReachDist, parameter_StayWithinDist);
	currentKI->setPointers(this, &controller);
}

ActionControlNode::ActionControlNode()
	: ControlNode()
{
	// SERVICES
	setReference_ = nh_.advertiseService("drone_autopilot/setReference", 
		&ActionControlNode::setReference, this);
	setMaxControl_ = nh_.advertiseService("drone_autopilot/setMaxControl", 
		&ActionControlNode::setMaxControl, this);
	setInitialReachDistance_ = nh_.advertiseService("drone_autopilot/setInitialReachDist", 
		&ActionControlNode::setInitialReachDist, this);
	setStayWithinDist_ = nh_.advertiseService("drone_autopilot/setStayWithinDist", 
		&ActionControlNode::setStayWithinDist, this);
	setStayTime_ = nh_.advertiseService("drone_autopilot/setStayTime", 
		&ActionControlNode::setStayTime, this);
	startControl_ = nh_.advertiseService("drone_autopilot/start", 
		&ActionControlNode::start, this);
	stopControl_ = nh_.advertiseService("drone_autopilot/stop", 
		&ActionControlNode::stop, this);
	clearCommands_ = nh_.advertiseService("drone_autopilot/clearCommands", 
		&ActionControlNode::clear, this);
	hover_ = nh_.advertiseService("drone_autopilot/hover", 
		&ActionControlNode::hover, this);
	lockScaleFP_ = nh_.advertiseService("drone_autopilot/lockScaleFP", 
		&ActionControlNode::lockScaleFP, this);
	// ACTIONS
	takeoff_ = new EmptyServer(nh_, "drone_autopilot/takeoff", false);
	takeoff_->registerGoalCallback(boost::bind(&ActionControlNode::takeoff, this));
	takeoff_->start();
	land_ = new EmptyServer(nh_, "drone_autopilot/land", false);
	land_->registerGoalCallback(boost::bind(&ActionControlNode::land, this));
	land_->start();
	autoInit_ = new AutoInitServer(nh_, "drone_autopilot/autoInit", false);
	autoInit_->registerGoalCallback(boost::bind(&ActionControlNode::autoInit, this));
	autoInit_->start();
	autoTakeover_ = new AutoInitServer(nh_, "drone_autopilot/autoTakeover", false);
	autoTakeover_->registerGoalCallback(boost::bind(&ActionControlNode::autoTakeover, this));
	autoTakeover_->start();
	goto_ = new MoveServer(nh_, "drone_autopilot/goto", false);
	goto_->registerGoalCallback(boost::bind(&ActionControlNode::goTo, this));
	goto_->start();
	moveBy_ = new MoveServer(nh_, "drone_autopilot/moveBy", false);
	moveBy_->registerGoalCallback(boost::bind(&ActionControlNode::moveBy, this));
	moveBy_->start();
	moveByRel_ = new MoveServer(nh_, "drone_autopilot/moveByRel", false);
	moveByRel_->registerGoalCallback(boost::bind(&ActionControlNode::moveByRel, this));
	moveByRel_->start();
}

ActionControlNode::~ActionControlNode()
{
	// ACTIONS
	delete autoInit_;
	delete autoTakeover_;
	delete takeoff_;
	delete land_;
	delete goto_;
	delete moveBy_;
	delete moveByRel_;
}

void ActionControlNode::updateControl(const tum_ardrone::filter_stateConstPtr state) {
	bool done = currentKI->update(state);
	if (done) {
		// which action?
		if (currentAction == "autoInit") autoInit_->setSucceeded();
		else if (currentAction == "autoTakeover") autoTakeover_->setSucceeded();
		else if (currentAction == "takeoff") takeoff_->setSucceeded();
		else if (currentAction == "land") land_->setSucceeded();
		else if (currentAction == "goto") goto_->setSucceeded();
		else if (currentAction == "moveBy") moveBy_->setSucceeded();
		else if (currentAction == "moveByRel") moveByRel_->setSucceeded();
		// delete
		delete currentKI;
		currentKI = NULL;
		currentAction = "";
	}
}

