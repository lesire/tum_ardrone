#ifndef __ACTIONCONTROLNODE_H
#define __ACTIONCONTROLNODE_H

#include "ControlNode.h"

#include "tum_ardrone/SetReference.h"
#include "tum_ardrone/SetMaxControl.h"

class ActionControlNode : public ControlNode
{
protected:
	ros::ServiceServer setReference_;
	ros::ServiceServer setMaxControl_;

	bool setReference(tum_ardrone::SetReference::Request&, tum_ardrone::SetReference::Response&);
	bool setMaxControl(tum_ardrone::SetMaxControl::Request&, tum_ardrone::SetMaxControl::Response&);

public:
	ActionControlNode();
	~ActionControlNode();

	// main pose-estimation loop
	void Loop();

};

#endif /* __ACTIONCONTROLNODE_H */
