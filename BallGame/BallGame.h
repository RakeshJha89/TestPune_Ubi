
#ifndef __BallGame_h_
#define __BallGame_h_

#include "BaseApplication.h"
#include "FollowerCamera.h"
#include "btBulletDynamicsCommon.h"


class BallGame : public BaseApplication
{
public:
    BallGame(void);
    virtual ~BallGame(void);

	bool isKeyboardEnabled, mJump, mGoingRight, mGoingLeft;
	Ogre::Real ballSpeed;
	Ogre::SceneNode* mRootNode, *mPlayerNode, *mPlaneNode;
	FollowerCamera* cam;

	//Physics related

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;
	
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	
	btDynamicsWorld*		m_dynamicsWorld;

	enum collisiontypes {
    COL_NOTHING = 0, //<Collide with nothing
    COL_FLOOR = BIT(0), //<Collide with ships
    COL_BALL = BIT(1), //<Collide with walls
    COL_POWERUP = BIT(2) //<Collide with powerups
	};
	UINT ballCollisionMask;
	UINT floorCollisionMask;
	
	//End


protected:
    virtual void createScene(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed(const OIS::KeyEvent&);
	virtual bool keyReleased(const OIS::KeyEvent&);
    virtual bool mouseMoved( const OIS::MouseEvent &arg );
    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

	void setupGame();
	void setupPhysics();
	void setupPhysicsObject();
	void updatePhysics();
	void deletePhysics();
	void createAmbientLight();
	void createEntities();
	void createCameraFollower();
	void updateBall(const Ogre::FrameEvent& evt);
};

#endif // #ifndef __BallGame_h_
