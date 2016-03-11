
#ifndef _FOLLOWER_CAMERA__
#define _FOLLOWER_CAMERA__

class FollowerCamera
{
public:
	FollowerCamera();
	void updateFollower(Ogre::Real deltaTime);
	FollowerCamera(Ogre::Camera* cam, Ogre::SceneNode *target);
	~FollowerCamera();
private:
	Ogre::Camera *_cam;
	Ogre::SceneNode *mCamPivotN, *mCamGoalN, *mCamN, *_target;
	void SetupFollower();
};

#endif