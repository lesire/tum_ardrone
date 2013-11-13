#ifndef __ACTIONCONTROLNODE_H
#define __ACTIONCONTROLNODE_H

#include "ControlNode.h"

#include "tum_ardrone/SetReference.h"
#include "tum_ardrone/SetMaxControl.h"
#include "tum_ardrone/SetInitialReachDistance.h"
#include "tum_ardrone/SetStayWithinDist.h"
#include "tum_ardrone/SetStayTime.h"
#include "std_srvs/Empty.h"

#include "tum_ardrone/AutoInitAction.h"
#include "tum_ardrone/EmptyAction.h"
#include "tum_ardrone/MoveAction.h"
#include <actionlib/server/simple_action_server.h>

class ActionControlNode : public ControlNode
{
protected:
	// SERVICES

	ros::ServiceServer setReference_;
	ros::ServiceServer setMaxControl_;
	ros::ServiceServer setInitialReachDistance_;
	ros::ServiceServer setStayWithinDist_;
	ros::ServiceServer setStayTime_;
	ros::ServiceServer startControl_;
	ros::ServiceServer stopControl_;
	ros::ServiceServer clearCommands_;
	ros::ServiceServer hover_;
	ros::ServiceServer lockScaleFP_;

	bool setReference(tum_ardrone::SetReference::Request&, tum_ardrone::SetReference::Response&);
	bool setMaxControl(tum_ardrone::SetMaxControl::Request&, tum_ardrone::SetMaxControl::Response&);
	bool setInitialReachDist(tum_ardrone::SetInitialReachDistance::Request&, tum_ardrone::SetInitialReachDistance::Response&);
	bool setStayWithinDist(tum_ardrone::SetStayWithinDist::Request&, tum_ardrone::SetStayWithinDist::Response&);
	bool setStayTime(tum_ardrone::SetStayTime::Request&, tum_ardrone::SetStayTime::Response&);
	bool start(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool clear(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool hover(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool lockScaleFP(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

	// ACTIONS
	std::string currentAction;

	typedef actionlib::SimpleActionServer<tum_ardrone::AutoInitAction> AutoInitServer;
	AutoInitServer *autoInit_, *autoTakeover_;
	void autoInit();
	void autoTakeover();

	typedef actionlib::SimpleActionServer<tum_ardrone::EmptyAction> EmptyServer;
	EmptyServer *takeoff_, *land_;
	void land();
	void takeoff();

	typedef actionlib::SimpleActionServer<tum_ardrone::MoveAction> MoveServer;
	MoveServer *goto_, *moveBy_, *moveByRel_;
	void goTo();
	void moveBy();
	void moveByRel();

public:
	ActionControlNode();
	~ActionControlNode();
	virtual void updateControl(const tum_ardrone::filter_stateConstPtr);
};

#endif /* __ACTIONCONTROLNODE_H */
