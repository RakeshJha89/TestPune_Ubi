
#include "stdafx.h"
#include "FollowerCamera.h"

FollowerCamera::FollowerCamera()
{

}
FollowerCamera::FollowerCamera(Ogre::Camera *cam, Ogre::SceneNode *target)
{
	_cam = cam;
	_target = target;

	SetupFollower();
}
FollowerCamera::~FollowerCamera()
{
	_cam = NULL;
}

void FollowerCamera::updateFollower(Ogre::Real deltaTime)
{
	mCamPivotN->setPosition(_target->getPosition() * Ogre::Vector3(1,0,1) + Ogre::Vector3::UNIT_Y * CAM_Y_OFFSET);
	Ogre::Vector3 destOffset = mCamGoalN->_getDerivedPosition() - mCamN->getPosition();
	mCamN->translate(destOffset * deltaTime * CAM_LERP_TIME);
	mCamN->lookAt(mCamPivotN->getPosition(),Ogre::Node::TS_WORLD);
}


void FollowerCamera::SetupFollower()
{
	mCamPivotN = _cam->getSceneManager()->getRootSceneNode()->createChildSceneNode(); // create the look at point.

	mCamGoalN = mCamPivotN->createChildSceneNode(Ogre::Vector3(0,0, CAM_Z_OFFSET)); // create an offset point for camera.

	mCamN = _cam->getSceneManager()->getRootSceneNode()->createChildSceneNode();
	mCamN->setPosition(mCamPivotN->getPosition() + mCamGoalN->getPosition()); // set the cam node to the offset point.

	mCamPivotN->setFixedYawAxis(true);
	mCamGoalN->setFixedYawAxis(true);
	mCamN->setFixedYawAxis(true);

	mCamN->attachObject(_cam);
	_cam->setNearClipDistance(0.1f);
	//_cam->setFarClipDistance(100);
}

/*

// place the camera pivot roughly at the character's shoulder
		mCameraPivot->setPosition(mBodyNode->getPosition() + Vector3::UNIT_Y * CAM_HEIGHT);
		// move the camera smoothly to the goal
		Vector3 goalOffset = mCameraGoal->_getDerivedPosition() - mCameraNode->getPosition();
		mCameraNode->translate(goalOffset * deltaTime * 9.0f);
		// always look at the pivot
		mCameraNode->lookAt(mCameraPivot->_getDerivedPosition(), Node::TS_WORLD);

// create a pivot at roughly the character's shoulder
		mCameraPivot = cam->getSceneManager()->getRootSceneNode()->createChildSceneNode();
		// this is where the camera should be soon, and it spins around the pivot
		mCameraGoal = mCameraPivot->createChildSceneNode(Vector3(0, 0, 15));
		// this is where the camera actually is
		mCameraNode = cam->getSceneManager()->getRootSceneNode()->createChildSceneNode();
		mCameraNode->setPosition(mCameraPivot->getPosition() + mCameraGoal->getPosition());

		mCameraPivot->setFixedYawAxis(true);
		mCameraGoal->setFixedYawAxis(true);
		mCameraNode->setFixedYawAxis(true);

		// our model is quite small, so reduce the clipping planes
		cam->setNearClipDistance(0.1);
		cam->setFarClipDistance(100);
		mCameraNode->attachObject(cam);

		mPivotPitch = 0;

*/