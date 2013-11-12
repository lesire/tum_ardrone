#ifndef __ACTIONCONTROLNODE_H
#define __ACTIONCONTROLNODE_H

#include "ControlNode.h"

#include "tum_ardrone/SetReference.h"
#include "tum_ardrone/SetMaxControl.h"
#include "tum_ardrone/SetInitialReachDistance.h"
#include "tum_ardrone/SetStayWithinDist.h"

class ActionControlNode : public ControlNode
{
protected:
	ros::ServiceServer setReference_;
	ros::ServiceServer setMaxControl_;
	ros::ServiceServer setInitialReachDistance_;
	ros::ServiceServer setStayWithinDist_;

	bool setReference(tum_ardrone::SetReference::Request&, tum_ardrone::SetReference::Response&);
	bool setMaxControl(tum_ardrone::SetMaxControl::Request&, tum_ardrone::SetMaxControl::Response&);
	bool setInitialReachDist(tum_ardrone::SetInitialReachDistance::Request&, tum_ardrone::SetInitialReachDistance::Response&);
	bool setStayWithinDist(tum_ardrone::SetStayWithinDist::Request&, tum_ardrone::SetStayWithinDist::Response&);

public:
	ActionControlNode();
	~ActionControlNode();

	// main pose-estimation loop
	void Loop();

};

#endif /* __ACTIONCONTROLNODE_H */
