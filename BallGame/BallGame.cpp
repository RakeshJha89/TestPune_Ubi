
#include "stdafx.h"
#include "BallGame.h"


//-------------------------------------------------------------------------------------
BallGame::BallGame(void)
{
	setupPhysics();
}
//-------------------------------------------------------------------------------------
BallGame::~BallGame(void)
{
	deletePhysics();
}

//-------------------------------------------------------------------------------------
void BallGame::createScene(void)
{
	 // create your scene here :)

	setupGame();
	
	createAmbientLight();
	
	createEntities();

	createCameraFollower();

	setupPhysicsObject();
}

void BallGame::setupGame()
{
	mRootNode = mSceneMgr->getRootSceneNode();// Get root node here.
	mGoingRight = mGoingLeft = mJump = false;
	isKeyboardEnabled = true;
	ballSpeed = BALL_SPEED;
}

void BallGame::setupPhysics()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld ->setDebugDrawer(&sDebugDrawer);
	m_dynamicsWorld->getSolverInfo().m_splitImpulse=true;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;
	m_dynamicsWorld->getDispatchInfo().m_useContinuous=false;
	m_dynamicsWorld->setGravity(btVector3(0,-1,0)); //-10 was default
	
}

void BallGame::setupPhysicsObject()
{
	//Setup physics masks
	
	ballCollisionMask = COL_FLOOR;
	floorCollisionMask = COL_BALL;

	///create a few basic rigid bodies


		// Create platform
	{
		btBoxShape* box = new btBoxShape(btVector3(btScalar(GROUND_WIDTH * 0.5f),btScalar(1.),btScalar(GROUND_DEPTH * 0.5f)));
		btCollisionShape* groundShape = box;
		
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body,COL_FLOOR,floorCollisionMask);
	}

	// Create Sphere player ball
	{
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* colShape = new btSphereShape(btScalar(SPHERE_RADIUS));
		m_collisionShapes.push_back(colShape);


		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

			startTransform.setOrigin(btVector3(0,SPHERE_RADIUS + 100,0));
		
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);
			body->setUserPointer((void*)mPlayerNode);

			m_dynamicsWorld->addRigidBody(body, COL_BALL, ballCollisionMask);
	}
}

struct btContactResCall : public btCollisionWorld::ContactResultCallback
{
	bool &testVal;
	btContactResCall(bool &boolVal):testVal(boolVal)
	{
	}
	virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
	{
		{
			Ogre::SceneNode *Node = static_cast<Ogre::SceneNode *>(colObj0->getUserPointer());
			if(Node)
				Node->setVisible(false);
		}
		{
			Ogre::SceneNode *Node = static_cast<Ogre::SceneNode *>(colObj1->getUserPointer());
			if(Node)
				Node->setVisible(false);
		}
		//testVal = true;
		return 0;
	}
};



void BallGame::updatePhysics()
{
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(btScalar(1./60.),7);//ms / 1000000.f);
		
		{
			//Player physics;
			btCollisionObject* Obj = m_dynamicsWorld->getCollisionObjectArray()[1];
			btRigidBody* Body = btRigidBody::upcast(Obj);
			Body->setActivationState(DISABLE_DEACTIVATION);
			btTransform trans;
			
			if(Body && Body->getMotionState())
			{
				Body->getMotionState()->getWorldTransform(trans);
				if(mGoingRight)
				{
					Body->setLinearVelocity(btVector3(BALL_SPEED,Body->getLinearVelocity().getY(),Body->getLinearVelocity().getZ()));
				}
				else if(mGoingLeft)
				{
					Body->setLinearVelocity(btVector3(-BALL_SPEED,Body->getLinearVelocity().getY(),Body->getLinearVelocity().getZ()));
				}
				else
				{
					Body->setLinearVelocity(btVector3(Body->getLinearVelocity().getX() * 0.05f, Body->getLinearVelocity().getY(), Body->getLinearVelocity().getZ()));
				}

				if(mJump)
				{
					Body->setLinearVelocity(btVector3(Body->getLinearVelocity().getX(),JUMP_SPEED,Body->getLinearVelocity().getZ()));
				}
			}

			if(Body)
			{
				mPlayerNode->setPosition(Ogre::Vector3(float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ())));
				mPlayerNode->setOrientation(Ogre::Real(trans.getRotation().getW()),Ogre::Real(trans.getRotation().getX()),Ogre::Real(trans.getRotation().getY()),Ogre::Real(trans.getRotation().getZ()));
			}
		}

		//callbacks.

		//btContactResCall mContactCall(mShutDown);
		//m_dynamicsWorld->contactPairTest(m_dynamicsWorld->getCollisionObjectArray()[0], m_dynamicsWorld->getCollisionObjectArray()[1], mContactCall);
	}

}

void BallGame::deletePhysics()
{
	if(!m_dynamicsWorld) return;
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void BallGame::createAmbientLight()
{
	//mSceneMgr->setAmbientLight(Ogre::ColourValue::White);
}

void BallGame::createEntities()
{
	Ogre::Entity* playerBall = mSceneMgr->createEntity("playerBall",Ogre::SceneManager::PT_SPHERE);
	mPlayerNode = mRootNode->createChildSceneNode("mPlayerNode");
	mPlayerNode->attachObject(playerBall); 

	Ogre::MaterialPtr playerBallMat = Ogre::MaterialManager::getSingleton().create("playerBallMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::TextureUnitState* playerTstate = playerBallMat->getTechnique(0)->getPass(0)->createTextureUnitState("spheremap.png");
	playerBall->setMaterialName("playerBallMat");

	mPlayerNode->setPosition(Ogre::Vector3(0,0,0));
	mPlayerNode->scale(Ogre::Vector3(0.1f,0.1f,0.1f));


	// create a floor mesh resource
	Ogre::MeshManager::getSingleton().createPlane("floor", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::Plane(Ogre::Vector3::UNIT_Y, 0), GROUND_WIDTH, GROUND_DEPTH, 10, 10, true, 1, 10, 10, Ogre::Vector3::UNIT_Z);
	Ogre::Entity* planeEnt = mSceneMgr->createEntity("LightPlaneEntity","floor");
	mPlaneNode = mRootNode->createChildSceneNode("mPlaneNode");
	mPlaneNode->attachObject(planeEnt);

	//Ogre::MaterialPtr planeMat = Ogre::MaterialManager::getSingleton().create("planeMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	//Ogre::TextureUnitState* planeTstate = planeMat->getTechnique(0)->getPass(0)->createTextureUnitState("GreenSkin.jpg");
	planeEnt->setMaterialName("Examples/Rockwall");
}

void BallGame::createCameraFollower()
{
	cam = new FollowerCamera(mCamera, mPlayerNode);

	// Create a direction light 
	Ogre::Light* light = mSceneMgr->createLight("MainLight");
	light->setType(Ogre::Light::LT_DIRECTIONAL);
	light->setPosition(Ogre::Vector3(0,500,10));
	light->setDirection(mPlayerNode->getPosition() - light->getPosition());
	//light->setDiffuseColour(Ogre::ColourValue(1.0f,1.0f,1.0f));
	

}

bool BallGame::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

	updatePhysics();
	
	if(cam != NULL)
		cam->updateFollower(evt.timeSinceLastFrame);

    return true;
}



bool BallGame::keyPressed(const OIS::KeyEvent &evt)
{
	if(!isKeyboardEnabled) return false;

	if (evt.key == OIS::KC_W || evt.key == OIS::KC_UP || evt.key == OIS::KC_SPACE) mJump = true;
	else if (evt.key == OIS::KC_A || evt.key == OIS::KC_LEFT) mGoingLeft = true;
	else if (evt.key == OIS::KC_D || evt.key == OIS::KC_RIGHT) mGoingRight = true;

	if (evt.key == OIS::KC_ESCAPE)
    {
        mShutDown = true;
    }
    return true;
}
bool BallGame::keyReleased(const OIS::KeyEvent &evt)
{
	if(!isKeyboardEnabled) return false;

	if (evt.key == OIS::KC_W || evt.key == OIS::KC_UP || evt.key == OIS::KC_SPACE) mJump = false;
	else if (evt.key == OIS::KC_A || evt.key == OIS::KC_LEFT) mGoingLeft = false;
	else if (evt.key == OIS::KC_D || evt.key == OIS::KC_RIGHT) mGoingRight = false;

	return true;
}
bool BallGame::mouseMoved(const OIS::MouseEvent &evt )
{
	return true;
}
bool BallGame::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id )
{
	return true;
}
bool BallGame::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id )
{
	return true;
}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int evtc, char *evtv[])
#endif
    {
        // Create application object
        BallGame app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
