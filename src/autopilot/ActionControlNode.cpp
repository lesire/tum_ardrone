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
}

ActionControlNode::~ActionControlNode()
{
}

void ActionControlNode::Loop() {
	ControlNode::Loop();
}

