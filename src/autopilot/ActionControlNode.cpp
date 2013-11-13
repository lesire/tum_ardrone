#include "ActionControlNode.h"

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

ActionControlNode::ActionControlNode()
	: ControlNode()
{
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
}

ActionControlNode::~ActionControlNode()
{
}

void ActionControlNode::Loop() {
	ControlNode::Loop();
}

