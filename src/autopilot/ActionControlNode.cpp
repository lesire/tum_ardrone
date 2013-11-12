#include "ActionControlNode.h"

using namespace tum_ardrone;

bool ActionControlNode::setReference(SetReference::Request& req,
	SetReference::Response& res)
{
	parameter_referenceZero = DronePosition(TooN::makeVector(req.x, req.y, req.z),
		req.heading);	
	res.status = true;
	return true;
}

bool ActionControlNode::setMaxControl(tum_ardrone::SetMaxControl::Request& req,
	tum_ardrone::SetMaxControl::Response& res)
{
	parameter_MaxControl = req.speed;
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
}

ActionControlNode::~ActionControlNode()
{
}

void ActionControlNode::Loop() {
	ControlNode::Loop();
}

