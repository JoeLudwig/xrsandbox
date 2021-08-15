/*
 *  Copyright 2019-2021 Diligent Graphics LLC
 *  Copyright 2015-2019 Egor Yusov
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  In no event and under no legal theory, whether in tort (including negligence), 
 *  contract, or otherwise, unless required by applicable law (such as deliberate 
 *  and grossly negligent acts) or agreed to in writing, shall any Contributor be
 *  liable for any damages, including any direct, indirect, special, incidental, 
 *  or consequential damages of any character arising as a result of this License or 
 *  out of the use or inability to use the software (including but not limited to damages 
 *  for loss of goodwill, work stoppage, computer failure or malfunction, or any and 
 *  all other commercial damages or losses), even if such Contributor has been advised 
 *  of the possibility of such damages.
 */

#include "xrappbase.h"

#include <memory>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <list>

#ifndef NOMINMAX
#	define NOMINMAX
#endif
#include <Windows.h>
#include <crtdbg.h>

#ifndef PLATFORM_WIN32
#	define PLATFORM_WIN32 1
#endif

#include <EngineFactory.h>

#include <EngineFactoryD3D11.h>
#include <EngineFactoryD3D12.h>
//#include <EngineFactoryVk.h>
#include <AdvancedMath.hpp>
#include <Timer.hpp>

#include <RenderDevice.h>
#include <DeviceContext.h>
#include <SwapChain.h>

#include <RefCntAutoPtr.hpp>

#include <MapHelper.hpp>
#include <GraphicsUtilities.h>
#include <GLTFLoader.hpp>
#include <GLTF_PBR_Renderer.hpp>
#include <TextureUtilities.h>
#include <openxr/openxr.h>

#include <PxPhysicsAPI.h>
#include <extensions\PxExtensionsAPI.h>

// Make sure the supported OpenXR graphics APIs are defined
#if D3D11_SUPPORTED
#include <d3d11.h>
#	define XR_USE_GRAPHICS_API_D3D11
#include <RenderDeviceD3D11.h>
#endif

#if D3D12_SUPPORTED
#include <d3d12.h>
#	define XR_USE_GRAPHICS_API_D3D12
#include <RenderDeviceD3D12.h>
#endif

#if GL_SUPPORTED
#	define XR_USE_GRAPHICS_API_OPENGL
#endif

#include <openxr/openxr_platform.h>
#include "graphics_utilities.h"
#include "igraphicsbinding.h"

#include "actions.h"
#include "paths.h"

namespace Diligent
{
#include <Shaders/Common/public/BasicStructures.fxh>
};

using namespace Diligent;
using namespace physx;
using namespace XRDE;


enum class Hand
{
	Left = 0,
	Right = 1,

	None = -1,
	Any = 100,
};

static const Hand BothHands[] = { Hand::Left, Hand::Right };

const char* toStr( Hand hand )
{
	switch ( hand )
	{
	case Hand::Left: return "Left";
	case Hand::Right: return "Right";
	case Hand::None: return "None";
	case Hand::Any: return "Any";
	default: return "UNKNOWN!";
	}
}

XrPath HandPath( Hand hand )
{
	switch ( hand )
	{
	default:
	case Hand::Left: return Paths().userHandLeft;
	case Hand::Right: return Paths().userHandRight;
	}
}

struct GLTFMeshGeometryPrimitive
{
	PxBoxGeometry geometry;
	Transform geoToNode;
};

struct GLTFMeshGeometry
{
	std::vector<GLTFMeshGeometryPrimitive> primitives;
	uint32_t nodeIndex = 0;
	float4x4 nodeToModelUnscaled;
};

typedef std::vector< std::unique_ptr< GLTFMeshGeometry > > GLTFMeshGeometryVector;

PxVec3 toPx( const float3& v )
{
	return { v.x, v.y, v.z };
}
PxQuat toPx( const Quaternion& q )
{
	return PxQuat( q.q.x, q.q.y, q.q.z, q.q.w );
}
PxTransform toPx( const Transform& t )
{
	return PxTransform( toPx( t.translation ), toPx( t.rotation ) );
}
PxTransform toPx( const float4x4 & m )
{
	PxMat44 out;
	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			out[ i ][ j ] = m[ i ][ j ];
		}
	}
	return PxTransform( out );
}

float3 toDE( const PxVec3& v )
{
	return float3( v.x, v.y, v.z );
}


Quaternion toDE( const PxQuat& q )
{
	return Quaternion( q.x, q.y, q.z, q.w );
}

Transform toDE( const PxTransform& pose )
{
	return Transform( toDE( pose.p ), toDE( pose.q ) );
}

inline uint32_t HandFlag( Hand hand )
{
	if ( hand == Hand::Any )
	{
		return HandFlag( Hand::Left ) | HandFlag( Hand::Right );
	}
	else
	{
		return 1 << static_cast<int>( hand );
	}
}

struct WorldObject
{
	Transform objectToWorld;
	std::unique_ptr<GLTF::Model> model;
	PxRigidDynamic* actor = nullptr;
	Hand grabbingHand = Hand::None;

	void clearHandTouch( Hand hand ) { flags = flags & ~HandFlag( hand ); }
	void addHandTouch( Hand hand ) { flags |= HandFlag( hand ); }
	bool hasHandTouch( Hand hand ) { return 0 != ( flags & HandFlag( hand ) ); }
protected:
	uint32_t flags = 0;



};

struct InputState
{
	Transform handToWorld;
	bool handToWorldValid = false;
	bool spawn = false;
	bool grab = false;
};

enum class HandState
{
	Idle,
	Highlight,
	Grab,
};

struct HandInfo
{
	Transform wristToWorld;
	WorldObject* touch = nullptr;
	HandState state = HandState::Idle;
	Transform grabbedObjectToPalm;
	PxRigidDynamic* grabActor = nullptr;
	PxFixedJoint* grabJoint = nullptr;
	PxArticulation* articulation = nullptr;
	std::unique_ptr<GLTF::Model> model;

	PxRigidDynamic* handCollider = nullptr;
	PxShape* colliderJoint[ XR_HAND_JOINT_COUNT_EXT ] = {};
	PxFixedJoint* colliderFixedJoint = nullptr;
	uint32_t handTouches = 0;
};


class XRSApp: public XrAppBase, public PxSimulationEventCallback
{
	typedef XrAppBase super;
public:
	XRSApp()
	{
	}

	virtual ~XRSApp()
	{
		if (m_pGraphicsBinding)
		{
			m_pGraphicsBinding->GetImmediateContext()->Flush();
		}
	}

	bool Initialize( HWND hWnd )
	{
		if ( !super::Initialize( hWnd ) )
			return false;

		if ( !InitPhysics() )
			return false;

		return true;
	}

	bool InitPhysics();

	PxArticulation* createHandArticulation( Hand hand, Transform* jointToParent, float *jointRadius );

	virtual void Render() override;
	virtual void Update( double currTime, double elapsedTime, XrTime displayTime ) override;
	virtual bool PreSession() override;
	virtual bool PostSession() override;
	virtual std::vector<std::string> GetDesiredExtensions();

	virtual bool RenderEye( int eye ) override;
	virtual void UpdateEyeTransforms( float4x4 eyeToProj, float4x4 stageToEye, XrView& view ) override;
	void UpdateHandPoses( Hand hand, XrTime displayTime );
	
	WorldObject* SpawnObject( const Transform& objectToWorld, const std::string& modelPath );

	// -------------- PxSimulationEventCallback --------------
	virtual void onConstraintBreak( PxConstraintInfo* /*constraints*/, PxU32 /*count*/ ) override;
	virtual void onWake( PxActor** /*actors*/, PxU32 /*count*/ ) override {}
	virtual void onSleep( PxActor** /*actors*/, PxU32 /*count*/ ) override {}
	virtual void onTrigger( PxTriggerPair* pairs, PxU32 count ) override;
	virtual void onAdvance( const PxRigidBody* const*, const PxTransform*, const PxU32 ) override {}
	virtual void onContact( const PxContactPairHeader& /*pairHeader*/, const PxContactPair* pairs, PxU32 count ) override;


private:
	PxRigidDynamic* CreateActorForModel( GLTF::Model* mode, const GLTFMeshGeometryVector& geos, const Transform& objectToWorld );
	InputState ReadInputState( Hand hand, XrTime displayTime );
	HandInfo& handInfo( Hand hand ) { return m_handInfo[ static_cast<size_t>( hand ) ]; }

	RefCntAutoPtr<IPipelineState>		 m_pPSO;
	RefCntAutoPtr<IShaderResourceBinding> m_pSRB;
	float4x4							  m_ViewToProj;
	InputState							m_handInput[ 2 ];
	HandInfo							m_handInfo[ 2 ];

	std::unique_ptr< XRDE::ActionSet > m_handActionSet;
	XRDE::Action * m_handAction;
	XRDE::Action * m_spawnAction;
	XRDE::Action * m_grabAction;
	XRDE::Action * m_hapticAction;

	std::vector< std::unique_ptr<WorldObject> > m_worldObjects;

	PxDefaultAllocator		m_physxAllocator;
	PxDefaultErrorCallback	m_physxErrorCallback;
	PxFoundation*			m_physxFoundation = nullptr;
	PxPhysics*				m_physxPhysics = nullptr;
	PxCooking*				m_physxCooking = nullptr;
	PxDefaultCpuDispatcher* m_physxDispatcher = nullptr;
	PxScene*				m_physxScene = nullptr;
	PxMaterial*				m_physxMaterial = nullptr;
	PxPvd*					m_physxPvd = nullptr;
	PxRigidStatic*			m_groundPlane = nullptr;
	std::map<std::string, GLTFMeshGeometryVector> m_physxGeometry;
};

std::unique_ptr<IApp> CreateApp()
{
	return std::make_unique<XRSApp>();
}


std::vector<std::string> XRSApp::GetDesiredExtensions() 
{ 
	return 
	{
		XR_EXT_HAND_TRACKING_EXTENSION_NAME,
		XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME,
//		XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME,
	}; 
}

std::unique_ptr<XRSApp> g_pTheApp;


bool XRSApp::PreSession()
{
	XrPath leftHand = StringToPath( m_instance, k_userHandLeft );
	XrPath rightHand = StringToPath( m_instance, k_userHandRight );

	m_handActionSet = std::make_unique<ActionSet>( "hands", "Hands", 0 );

	m_handAction = m_handActionSet->AddAction( "handpose", "Hand Location", XR_ACTION_TYPE_POSE_INPUT, 
		std::vector( { leftHand, rightHand } ) );
	m_handAction->AddGlobalBinding( Paths().rightGripPose );
	m_handAction->AddGlobalBinding( Paths().leftGripPose );

	m_spawnAction = m_handActionSet->AddAction( "spawn", "Spawn Object", XR_ACTION_TYPE_BOOLEAN_INPUT,
		std::vector( { leftHand, rightHand } ) );
	m_spawnAction->AddIPBinding( Paths().interactionProfilesValveIndexController, Paths().leftAClick );
	m_spawnAction->AddIPBinding( Paths().interactionProfilesOculusTouchController, Paths().leftXClick );
	m_spawnAction->AddGlobalBinding( Paths().rightAClick );

	m_grabAction = m_handActionSet->AddAction( "grab", "Grab Object", XR_ACTION_TYPE_BOOLEAN_INPUT,
		std::vector( { leftHand, rightHand } ) );
	m_grabAction->AddGlobalBinding( Paths().rightTrigger );
	m_grabAction->AddGlobalBinding( Paths().leftTrigger );

	m_hapticAction = m_handActionSet->AddAction( "haptics", "Cube Haptics", XR_ACTION_TYPE_VIBRATION_OUTPUT,
		std::vector( { leftHand, rightHand } ) );
	m_hapticAction->AddGlobalBinding( Paths().rightHaptic);
	m_hapticAction->AddGlobalBinding( Paths().leftHaptic );

	CHECK_XR_RESULT( m_handActionSet->Init( m_instance ) );

	std::vector<const ActionSet*> actionSets(
		{
			&*m_handActionSet,
		} );

	CHECK_XR_RESULT( SuggestBindings( m_instance, Paths().interactionProfilesValveIndexController, actionSets ) );

	return true;
}


bool XRSApp::PostSession()
{
	CHECK_XR_RESULT( AttachActionSets( m_session, { &*m_handActionSet } ) );

	CHECK_XR_RESULT( m_handActionSet->SessionInit( m_session ) );

	SetPbrEnvironmentMap( *m_gltfRenderer, "textures/papermill.ktx" );
	SetPbrEnvironmentMap( *m_highlightRenderer, "textures/highlight_hdr.ktx" );

	handInfo( Hand::Left ).model = LoadGltfModel( "models/valve_hand_models/left_hand.glb" );
	handInfo( Hand::Right ).model = LoadGltfModel( "models/valve_hand_models/right_hand.glb" );

	return true;
}

void setActorName( PxActor* actor, const std::string& name )
{
	static std::list<std::string> names;
	auto i = names.insert( names.begin(), name );
	actor->setName( i->c_str() );
}

enum class FilterObjectType
{
	Misc = 0,
	HandCollider = 1,
};

struct FilterDataHelper
{
	FilterDataHelper( const PxFilterData& data ) : filterData( data ) {};
	FilterDataHelper( FilterObjectType objType, Hand hand )
	{
		filterData.word0 = static_cast<uint32_t>( objType );
		filterData.word1 = static_cast<uint32_t>( hand );
	}

	FilterObjectType getObjectType() const { return static_cast<FilterObjectType>( filterData.word0 ); }
	Hand getHand() const 
	{
		if ( getObjectType() == FilterObjectType::HandCollider )
			return static_cast<Hand>( filterData.word1 );
		else
			return Hand::None;
	}

	operator PxFilterData& ( ) { return filterData;  }
	operator const PxFilterData& ( ) const { return filterData; }

	PxFilterData filterData;
};

bool XRSApp::InitPhysics()
{
	m_physxFoundation = PxCreateFoundation( PX_PHYSICS_VERSION, m_physxAllocator, m_physxErrorCallback );

	m_physxPvd = PxCreatePvd( *m_physxFoundation );
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate( "127.0.0.1", 5425, 10 );
	m_physxPvd->connect( *transport, PxPvdInstrumentationFlag::eALL );

	m_physxCooking = PxCreateCooking( PX_PHYSICS_VERSION, *m_physxFoundation, PxCookingParams( PxTolerancesScale() ) );
	m_physxPhysics = PxCreatePhysics( PX_PHYSICS_VERSION, *m_physxFoundation, PxTolerancesScale(), true, m_physxPvd );

	PxInitExtensions( *m_physxPhysics, m_physxPvd );

	PxSceneDesc sceneDesc( m_physxPhysics->getTolerancesScale() );
	sceneDesc.gravity = PxVec3( 0.0f, -9.81f, 0.0f );
	m_physxDispatcher = PxDefaultCpuDispatcherCreate( 2 );
	sceneDesc.cpuDispatcher = m_physxDispatcher;
	sceneDesc.filterShader = []( PxFilterObjectAttributes attributes0, PxFilterData filterData0,
			PxFilterObjectAttributes attributes1, PxFilterData filterData1,
			PxPairFlags & pairFlags, const void* constantBlock, PxU32 constantBlockSize ) ->PxFilterFlags
	{
		if ( FilterDataHelper( filterData0 ).getObjectType() == FilterObjectType::HandCollider
			|| FilterDataHelper( filterData1 ).getObjectType() == FilterObjectType::HandCollider )
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT | PxPairFlag::eCONTACT_DEFAULT;
			return PxFilterFlag::eDEFAULT;
		}

		// otherwise, we want default behavior
		return PxDefaultSimulationFilterShader( attributes0, filterData0, attributes1, filterData1, pairFlags, 
			constantBlock, constantBlockSize );
	};
	//sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.simulationEventCallback = this;
	m_physxScene = m_physxPhysics->createScene( sceneDesc );

	PxPvdSceneClient* pvdClient = m_physxScene->getScenePvdClient();
	if ( pvdClient )
	{
		pvdClient->setScenePvdFlag( PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true );
		pvdClient->setScenePvdFlag( PxPvdSceneFlag::eTRANSMIT_CONTACTS, true );
		pvdClient->setScenePvdFlag( PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true );
	}
	m_physxMaterial = m_physxPhysics->createMaterial( 0.5f, 0.5f, 0.6f );

	m_groundPlane = PxCreatePlane( *m_physxPhysics, PxPlane( 0, 1, 0, -1.f ), *m_physxMaterial );
	m_groundPlane->setName( "Ground Plane" );
	m_physxScene->addActor( *m_groundPlane );

	for ( Hand hand : BothHands )
	{
		PxSphereGeometry sphere( 0.001f );
		PxShape* shape = m_physxPhysics->createShape( sphere, *m_physxMaterial, true, PxShapeFlag::eVISUALIZATION );
		HandInfo& handInfo = m_handInfo[ static_cast<int>( hand ) ];
		handInfo.grabActor = m_physxPhysics->createRigidDynamic( toPx( Transform() ) );
		handInfo.grabActor->setRigidBodyFlag( PxRigidBodyFlag::eKINEMATIC, true );
		handInfo.grabActor->attachShape( *shape );
		setActorName( handInfo.grabActor, std::string( "Grab Actor " ) + toStr( hand ) );

		m_physxScene->addActor( *handInfo.grabActor );
	}

	m_physxScene->setVisualizationParameter( PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f );
	m_physxScene->setVisualizationParameter( PxVisualizationParameter::eJOINT_LIMITS, 1.0f );

	return true;
}


void XRSApp::UpdateEyeTransforms( float4x4 eyeToProj, float4x4 stageToEye, XrView& view )
{
	// Map the buffer and write current world-view-projection matrix
	m_ViewToProj = eyeToProj;
};


bool XRSApp::RenderEye( int eye )
{
	m_gltfRenderer->Begin( m_pGraphicsBinding->GetRenderDevice(), m_pGraphicsBinding->GetImmediateContext(),
		m_CacheUseInfo, m_CacheBindings, m_CameraAttribsCB, m_LightAttribsCB );

	// draw the hands if they're available
	for ( Hand hand : BothHands )
	{
		int i = static_cast<int>( hand );
		if ( !m_handInput[ i ].handToWorldValid )
			continue;

		GLTF_PBR_Renderer::RenderInfo renderInfo;
		renderInfo.ModelTransform = toDE( handInfo( hand ).handCollider->getGlobalPose() ).toMatrix();
		m_gltfRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *m_handInfo[i].model, renderInfo,
			nullptr, &m_CacheBindings );
	}

	for ( auto& worldObject : m_worldObjects )
	{
		GLTF_PBR_Renderer::RenderInfo renderInfo;
		renderInfo.ModelTransform = worldObject->objectToWorld.toMatrix();
		if ( !worldObject->hasHandTouch( Hand::Any ) )
		{
			renderInfo.ModelTransform = float4x4::Scale( 1.1f ) * worldObject->objectToWorld.toMatrix();
			m_gltfRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *worldObject->model, renderInfo, nullptr, &m_CacheBindings );
		}

	}

	m_highlightRenderer->Begin( m_pGraphicsBinding->GetRenderDevice(), m_pGraphicsBinding->GetImmediateContext(),
		m_CacheUseInfo, m_highlightCacheBindings, m_CameraAttribsCB, m_LightAttribsCB );

	for ( auto& worldObject : m_worldObjects )
	{
		GLTF_PBR_Renderer::RenderInfo renderInfo;
		renderInfo.ModelTransform = worldObject->objectToWorld.toMatrix();
		if ( worldObject->hasHandTouch( Hand::Any ) )
		{
			renderInfo.ModelTransform = float4x4::Scale( 1.1f ) * worldObject->objectToWorld.toMatrix();
			m_highlightRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *worldObject->model, renderInfo, nullptr, &m_highlightCacheBindings );
		}
	}

	return true;
}


// Render a frame
void XRSApp::Render()
{
}


InputState XRSApp::ReadInputState( Hand hand, XrTime displayTime )
{
	XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
	InputState state;
	if ( XR_SUCCEEDED( m_handAction->LocateSpace( m_stageSpace, displayTime, HandPath( hand ), &spaceLocation ) )
		&& ( spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT ) != 0 )
	{
		state.handToWorld = toDE( spaceLocation.pose );
		state.handToWorldValid = true;
	}

	state.spawn = m_spawnAction->GetBooleanState( m_session, HandPath( hand ) );
	state.grab = m_grabAction->GetBooleanState( m_session, HandPath( hand ) );

	return state;
}

bool isBroken( PxJoint* joint )
{
	return joint->getConstraintFlags().isSet( PxConstraintFlag::eBROKEN );
}

void XRSApp::Update( double CurrTime, double ElapsedTime, XrTime displayTime )
{
	// never simulate more than a tenth of a second, just in case we were at a breakpoint
	m_physxScene->simulate( std::min( 0.1f, (float)ElapsedTime ) );
	m_physxScene->fetchResults( true );

	XrActiveActionSet activeActionSets[] =
	{
		{ m_handActionSet->Handle(), Paths().userHandLeft },
		{ m_handActionSet->Handle(), Paths().userHandRight },
	};
	XrActionsSyncInfo syncInfo = { XR_TYPE_ACTIONS_SYNC_INFO };
	syncInfo.activeActionSets = activeActionSets;
	syncInfo.countActiveActionSets = sizeof( activeActionSets ) / sizeof( activeActionSets[ 0 ] );
	xrSyncActions( m_session, &syncInfo );

	InputState oldState[ 2 ] = { m_handInput[ 0 ], m_handInput[ 1 ] };

	PxSphereGeometry sphere( 0.05f );

	for ( Hand hand : BothHands )
	{
		int i = static_cast<int>( hand );
		InputState oldState = m_handInput[ i ];
		InputState& handInput = m_handInput[ i ];
		HandInfo& handInfo = m_handInfo[ i ];
		handInput = ReadInputState( hand, displayTime );

		// update the grab actor's transform, if we have a new valid transform
		if ( handInput.handToWorldValid )
		{
			handInfo.grabActor->setKinematicTarget( toPx( handInfo.wristToWorld ) );
		}

		switch ( handInfo.state )
		{
		case HandState::Highlight:
		case HandState::Idle:
			if ( handInput.handToWorldValid )
			{
				if ( !oldState.spawn && handInput.spawn && handInput.handToWorldValid )
				{
					m_hapticAction->ApplyHapticFeedback( m_session, Paths().userHandRight, 0, 20, 1 );
					SpawnObject( m_handInfo[ i ].wristToWorld, "models/gear.glb" );
				}

				WorldObject* touchedObject = nullptr;
				PxOverlapBufferN<10> hit;
				PxQueryFilterData filterData( PxQueryFlag::eDYNAMIC );
				WorldObject* newTouch = nullptr;
				if ( m_physxScene->overlap( sphere, toPx( handInfo.wristToWorld ), hit, filterData ) && hit.hasAnyHits() )
				{
					newTouch = static_cast<WorldObject*>( hit.getTouch( 0 ).actor->userData );
				}

				if ( newTouch != handInfo.touch )
				{
					if ( handInfo.touch )
					{
						handInfo.touch->clearHandTouch( hand );
						handInfo.touch = nullptr;
						handInfo.state = HandState::Idle;
					}
					if ( newTouch )
					{
						handInfo.state = HandState::Highlight;
						handInfo.touch = newTouch;
						newTouch->addHandTouch( hand );
					}
				}

				if ( !oldState.grab && handInput.grab && newTouch )
				{
					// start grabbing
					if ( newTouch->grabbingHand != Hand::None && newTouch->grabbingHand != hand )
					{
						HandInfo& otherHand = m_handInfo[ static_cast<int>( newTouch->grabbingHand ) ];
						otherHand.state = HandState::Highlight;
						newTouch->grabbingHand = Hand::None;
					}

					newTouch->grabbingHand = hand;
					Transform objectToWorld = toDE( newTouch->actor->getGlobalPose() );
					Transform worldToWrist = handInfo.wristToWorld.inverse();
					handInfo.grabbedObjectToPalm = objectToWorld * worldToWrist;
					handInfo.state = HandState::Grab;

					handInfo.grabJoint = PxFixedJointCreate( *m_physxPhysics,
						handInfo.grabActor, toPx( Transform() ),
						newTouch->actor, toPx( handInfo.grabbedObjectToPalm.inverse() ) );
					static float force = 2.f;
					static float torque = 0.1f;
					handInfo.grabJoint->setBreakForce( force, torque );
					handInfo.grabJoint->setConstraintFlag( PxConstraintFlag::eVISUALIZATION, true );
				}
			}
			break;

		case HandState::Grab:
			if ( !handInput.grab || !handInfo.grabJoint || isBroken( handInfo.grabJoint ) )
			{
				if ( handInfo.touch && handInfo.touch->grabbingHand == hand )
				{
					handInfo.touch->grabbingHand = Hand::None;
				}
				if ( handInfo.grabJoint )
				{
					handInfo.grabJoint->release();
					handInfo.grabJoint = nullptr;
				}
				handInfo.state = HandState::Highlight;
			}
			break;
		}


		if ( handInfo.colliderFixedJoint )
		{
			if ( isBroken( handInfo.colliderFixedJoint ) )
			{
				OutputDebugString( L"Broken\n" );
				// if the collider fixed joint is broken, get rid of it. We'll be kept sort of in
				// place by the springs
				handInfo.colliderFixedJoint->release();
				handInfo.colliderFixedJoint = nullptr;
			}
			else
			{
				// everything is fine.
			}
		}
		else
		{
			// we're using the springs to position the hand. See if we can switch back to the fixed joint
			if ( !handInfo.handTouches )
			{
				OutputDebugString( L"Fixed\n" );
				handInfo.colliderFixedJoint = PxFixedJointCreate( *m_physxPhysics,
					handInfo.grabActor, toPx( Transform() ),
					handInfo.handCollider, toPx( Transform() ) );
				handInfo.colliderFixedJoint->setBreakForce( 5.f, 5.f );
			}
		}
	}

	UpdateHandPoses( Hand::Left, displayTime );
	UpdateHandPoses( Hand::Right, displayTime );


	// should look at using PxActor** activeActors = scene.getActiveActors(nbActiveActors);
	// to avoid looping over the whole list when most are sleeping
	for ( auto& obj : m_worldObjects )
	{
		if ( !obj->actor )
			continue;

		obj->objectToWorld = toDE( obj->actor->getGlobalPose() );
	}
}

XrHandJointEXT GetParentJoint( XrHandJointEXT joint )
{
	switch ( joint )
	{
		case XR_HAND_JOINT_PALM_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_WRIST_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_THUMB_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_THUMB_PROXIMAL_EXT: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
		case XR_HAND_JOINT_THUMB_DISTAL_EXT: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
		case XR_HAND_JOINT_THUMB_TIP_EXT: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
		case XR_HAND_JOINT_INDEX_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_INDEX_PROXIMAL_EXT: return XR_HAND_JOINT_INDEX_METACARPAL_EXT;
		case XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
		case XR_HAND_JOINT_INDEX_DISTAL_EXT: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
		case XR_HAND_JOINT_INDEX_TIP_EXT: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
		case XR_HAND_JOINT_MIDDLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT: return XR_HAND_JOINT_MIDDLE_METACARPAL_EXT;
		case XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
		case XR_HAND_JOINT_MIDDLE_DISTAL_EXT: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
		case XR_HAND_JOINT_MIDDLE_TIP_EXT: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
		case XR_HAND_JOINT_RING_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_RING_PROXIMAL_EXT: return XR_HAND_JOINT_RING_METACARPAL_EXT;
		case XR_HAND_JOINT_RING_INTERMEDIATE_EXT: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
		case XR_HAND_JOINT_RING_DISTAL_EXT: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
		case XR_HAND_JOINT_RING_TIP_EXT: return XR_HAND_JOINT_RING_DISTAL_EXT;
		case XR_HAND_JOINT_LITTLE_METACARPAL_EXT: return XR_HAND_JOINT_WRIST_EXT;
		case XR_HAND_JOINT_LITTLE_PROXIMAL_EXT: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
		case XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
		case XR_HAND_JOINT_LITTLE_DISTAL_EXT: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
		case XR_HAND_JOINT_LITTLE_TIP_EXT: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;

		default:
			return XR_HAND_JOINT_MAX_ENUM_EXT;
	}
}

uint32_t JointIndexFromHandJoint( XrHandJointEXT handJoint )
{
	if ( handJoint == XR_HAND_JOINT_PALM_EXT )
	{
		return 25;
	}
	else
	{
		return handJoint - 1;
	}
}


PxArticulation* XRSApp::createHandArticulation( Hand hand, Transform *jointToParent, float *jointRadius )
{
	PxArticulation* articulation = m_physxPhysics->createArticulation();
	//PxArticulationLink* links[ XR_HAND_JOINT_COUNT_EXT ] = {};

	//// make the root joint before looping because of the dumb joint order
	//links[ XR_HAND_JOINT_WRIST_EXT ] = articulation->createLink( nullptr, toPx( Transform() ) );
	//for ( XrHandJointEXT joint = XR_HAND_JOINT_PALM_EXT; joint < XR_HAND_JOINT_MAX_ENUM_EXT; ((int&)joint)++ )
	//{
	//	if ( joint == XR_HAND_JOINT_WRIST_EXT )
	//		continue;

	//	PxArticulationLink * parent = links[ GetParentJoint( joint ) ];
	//	links[ joint ] = articulation->createLink( parent, )
	//}
	//PxArticulationLink* link = articulation->createLink( parent, linkPose );
	//PxRigidActorExt::createExclusiveShape( *link, linkGeometry, material );
	//PxRigidBodyExt::updateMassAndInertia( *link, 1.0f );
	return articulation;
}

void XRSApp::UpdateHandPoses( Hand hand, XrTime displayTime )
{
	if ( !m_enableHandTrackers )
		return;

	HandInfo& handInfo = m_handInfo[ static_cast<int>( hand ) ];
	XrHandJointsLocateInfoEXT locateInfo = { XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT };
	locateInfo.time = displayTime;
	locateInfo.baseSpace = m_stageSpace;

	XrHandJointLocationEXT jointLocations[ XR_HAND_JOINT_COUNT_EXT ];
	XrHandJointLocationsEXT locations = { XR_TYPE_HAND_JOINT_LOCATIONS_EXT };
	locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
	locations.jointLocations = jointLocations;
	XrResult res = m_xrLocateHandJointsEXT( m_handTrackers[ static_cast<int>( hand ) ], &locateInfo, &locations );
	if ( XR_FAILED( res ) )
		return;

	if ( !locations.isActive )
		return;

	float jointRadius[ XR_HAND_JOINT_COUNT_EXT ];
	Transform jointsToParent[ XR_HAND_JOINT_COUNT_EXT ];
	Transform stageToJoint[ XR_HAND_JOINT_COUNT_EXT ];
	Transform jointToStage[ XR_HAND_JOINT_COUNT_EXT ];

	// pre-load the wrist because the palm is out of order and earlier in the enum
	jointsToParent[ XR_HAND_JOINT_WRIST_EXT ] = Transform();
	jointToStage[ XR_HAND_JOINT_WRIST_EXT ] = toDE( jointLocations[ XR_HAND_JOINT_WRIST_EXT ].pose );
	stageToJoint[ XR_HAND_JOINT_WRIST_EXT ] = jointToStage[ XR_HAND_JOINT_WRIST_EXT ].inverse();
	jointRadius[ XR_HAND_JOINT_WRIST_EXT ] = jointLocations[ XR_HAND_JOINT_WRIST_EXT ].radius;
	for ( uint32_t jointIndex = 0; jointIndex < XR_HAND_JOINT_COUNT_EXT; jointIndex++ )
	{
		if ( jointIndex == XR_HAND_JOINT_WRIST_EXT )
			continue;

		jointRadius[ jointIndex ] = jointLocations[ jointIndex ].radius;

		jointToStage[ jointIndex ] = toDE( jointLocations[ jointIndex ].pose );
		//float4x4 jointToStage = Diligent::float4x4::Translation( vectorFromXrVector( jointLocations[ jointIndex ].pose.position ) );
		stageToJoint[ jointIndex ] = jointToStage[ jointIndex ].inverse();

		XrHandJointEXT parentJoint = GetParentJoint( ( XrHandJointEXT)jointIndex );
		if ( parentJoint == jointIndex )
		{
			// this joint has no parent, so its parent is the model
			jointsToParent[ jointIndex ] = Transform();
		}
		else
		{
			jointsToParent[ jointIndex ] = jointToStage[ jointIndex ] * stageToJoint[ parentJoint ];
		}
	}

	for ( auto& skin : handInfo.model->Skins )
	{
		for ( uint32_t handJoint = 0; handJoint < 26; handJoint++ )
		{
			//if ( handJoint > XR_HAND_JOINT_THUMB_TIP_EXT )
			//	break;

			uint32_t jointIndex = JointIndexFromHandJoint( (XrHandJointEXT)handJoint );

			GLTF::Node* node = skin->Joints[ jointIndex ];
			node->Matrix = float4x4::Identity();
			node->Rotation = jointsToParent[ handJoint ].rotation;
			node->Translation = jointsToParent[ handJoint ].translation;
			node->Scale = { 1.f, 1.f, 1.f };
		}
	}

	for ( auto& root_node : handInfo.model->Nodes )
	{
		root_node->UpdateTransforms();
	}

	if ( !handInfo.handCollider )
	{
		handInfo.handCollider = m_physxPhysics->createRigidDynamic( toPx( jointToStage[ XR_HAND_JOINT_WRIST_EXT ]) );
		setActorName( handInfo.handCollider, std::string( "Collider " ) + toStr( hand ) );

		for ( uint32_t jointIndex = 0; jointIndex < XR_HAND_JOINT_COUNT_EXT; jointIndex++ )
		{
			handInfo.colliderJoint[ jointIndex ] = m_physxPhysics->createShape( PxSphereGeometry( jointRadius[ jointIndex ] ),
				*m_physxMaterial, true );
			handInfo.colliderJoint[ jointIndex ]->setSimulationFilterData( 
				FilterDataHelper( FilterObjectType::HandCollider, hand ) );
			handInfo.handCollider->attachShape( *handInfo.colliderJoint[ jointIndex ] );
		}

		handInfo.handCollider->setActorFlag( PxActorFlag::eDISABLE_GRAVITY, true );

		PxRigidBodyExt::updateMassAndInertia( *handInfo.handCollider, 100.0f );
		m_physxScene->addActor( *handInfo.handCollider );

		handInfo.colliderFixedJoint = PxFixedJointCreate( *m_physxPhysics,
				handInfo.grabActor, toPx( Transform() ),
				handInfo.handCollider, toPx( Transform() ) );
		handInfo.colliderFixedJoint->setBreakForce( 5.f, 5.f );

		float springStiffness = 10000.f;
		float springDamping = 10000000.f;

		PxDistanceJoint * handJoint = PxDistanceJointCreate( *m_physxPhysics,
			handInfo.grabActor, toPx( Transform() ),
			handInfo.handCollider, toPx( Transform() ) );
		handJoint->setDistanceJointFlag( PxDistanceJointFlag::eSPRING_ENABLED, true );
		handJoint->setStiffness( springStiffness );
		handJoint->setDamping( springDamping );
		handJoint->setTolerance( 0.01f );

		handJoint = PxDistanceJointCreate( *m_physxPhysics,
			handInfo.grabActor, toPx( Transform( float3( 0.1f, 0, 0 ) ) ),
			handInfo.handCollider, toPx( Transform( float3( 0.1f, 0, 0 ) ) ) );
		handJoint->setDistanceJointFlag( PxDistanceJointFlag::eSPRING_ENABLED, true );
		handJoint->setStiffness( springStiffness );
		handJoint->setDamping( springDamping );
		handJoint->setTolerance( 0.01f );

		handJoint = PxDistanceJointCreate( *m_physxPhysics,
			handInfo.grabActor, toPx( Transform( float3( 0, 0, -0.1f ) ) ),
			handInfo.handCollider, toPx( Transform( float3( 0, 0, -0.1f ) ) ) );
		handJoint->setDistanceJointFlag( PxDistanceJointFlag::eSPRING_ENABLED, true );
		handJoint->setStiffness( springStiffness );
		handJoint->setDamping( springDamping );
		handJoint->setTolerance( 0.01f );
	}

	// update the hand collider joints
	for ( uint32_t jointIndex = 0; jointIndex < XR_HAND_JOINT_COUNT_EXT; jointIndex++ )
	{
		handInfo.colliderJoint[ jointIndex ]->setLocalPose( toPx( jointToStage[ jointIndex ] * stageToJoint[ XR_HAND_JOINT_WRIST_EXT ] ) );
	}

	handInfo.wristToWorld = toDE( jointLocations[ XR_HAND_JOINT_WRIST_EXT ].pose );
}

// TODO:
//move hand position to be more central so that it doesn't rotate around the wrist?
//Slave the model rendering to the collider position so I can see what it's doing
//Maybe a fixed joint is too rigid? Would a distance joint be too weak?
//would help if the plane were rendered so I could see what I was colliding with
//a spawnable model with some height would HELP too
//spawned objects should probably spawn a bit away from the actual Hand so they don't immediately shoot away because of the interpenetration
//Need to figure out how to get onContact to be called

GLTFMeshGeometryVector GLTFMeshPhysXGeometry( GLTF::Model* model )
{
	GLTFMeshGeometryVector geos;
	for ( auto* node : model->LinearNodes )
	{
		if ( !node->pMesh )
			continue;

		// figure out the scale so we can back it out of everything
		float4x4 nodeToModel = node->GetMatrix();
		float xScale = length( float3( 1, 0, 0 ) * nodeToModel );
		float yScale = length( float3( 0, 1, 0 ) * nodeToModel );
		float zScale = length( float3( 0, 0, 1 ) * nodeToModel );
		float3 scale( xScale, yScale, zScale );

		std::unique_ptr<GLTFMeshGeometry> geo = std::make_unique<GLTFMeshGeometry>();
		geo->nodeIndex = node->Index;
		geo->nodeToModelUnscaled = float4x4::Scale( scale ).Inverse() * nodeToModel;

		for ( auto& prim : node->pMesh->Primitives )
		{
			float3 max = prim.BB.Max * scale;
			float3 min = prim.BB.Min * scale;
			float3 size = max - min;
			float3 halfSize = size / 2.f;
			float3 offset = halfSize + min;

			GLTFMeshGeometryPrimitive geoPrim;
			geoPrim.geometry = PxBoxGeometry( toPx( halfSize ) );
			geoPrim.geoToNode = Transform( offset );
			geo->primitives.push_back( geoPrim );
		}

		if ( !geo->primitives.empty() )
		{
			geos.push_back( std::move( geo ) );
		}
	}

	return std::move( geos );
}


PxRigidDynamic * XRSApp::CreateActorForModel( GLTF::Model* model, const GLTFMeshGeometryVector& geos, const Transform & objectToWorld )
{
	PxRigidDynamic* body = m_physxPhysics->createRigidDynamic( toPx( objectToWorld ) );

	for ( auto& geo : geos )
	{
		if ( geo->nodeIndex >= model->LinearNodes.size() )
			continue;

		GLTF::Node* node = model->LinearNodes[ geo->nodeIndex ];
		//ASSERT( node->Index == geo->nodeIndex );

		for ( auto& prim : geo->primitives )
		{
			PxShape* shape = m_physxPhysics->createShape( prim.geometry, *m_physxMaterial, true );
			shape->setLocalPose( toPx( prim.geoToNode.toMatrix() * geo->nodeToModelUnscaled ) );
			body->attachShape( *shape );
			shape->release();
		}
	}

	setActorName( body, std::string( "Spawn " ) + std::to_string( m_worldObjects.size() ) );

	PxRigidBodyExt::updateMassAndInertia( *body, 10.0f );
	m_physxScene->addActor( *body );

	return body;
}

WorldObject* XRSApp::SpawnObject( const Transform& objectToWorld, const std::string& modelPath )
{
	auto worldObject = std::make_unique<WorldObject>();
	worldObject->model = LoadGltfModel( modelPath );
	if ( !worldObject->model )
		return nullptr;

	auto iGeo = m_physxGeometry.find( modelPath );
	if ( iGeo == m_physxGeometry.end() )
	{
		auto geos = GLTFMeshPhysXGeometry( worldObject->model.get() );
		m_physxGeometry[ modelPath ] = std::move( geos );
		iGeo = m_physxGeometry.find( modelPath );
	}

	worldObject->actor = CreateActorForModel( worldObject->model.get(), iGeo->second, objectToWorld );
	worldObject->objectToWorld = objectToWorld;
	WorldObject* p = worldObject.get();
	worldObject->actor->userData = p;
	m_worldObjects.push_back( std::move( worldObject ) );
	return p;
}


// -------------- PxSimulationEventCallback --------------
void XRSApp::onConstraintBreak( PxConstraintInfo* constraints, PxU32 count )
{
	//for ( uint32_t c = 0; c < count; c++ )
	//{
	//	PxConstraintInfo& constraint = constraints[ c ];
	//	switch( constraint.type )
	//}
}

void XRSApp::onTrigger( PxTriggerPair* pairs, PxU32 count )
{
	for ( uint32_t i = 0; i < count; i++ )
	{
		PxTriggerPair& pair= pairs[ i ];
		if ( pair.status & PxPairFlag::eNOTIFY_TOUCH_FOUND )
		{
			for ( Hand hand : BothHands )
			{
				if ( handInfo( hand ).handCollider == pair.triggerActor || handInfo( hand ).handCollider == pair.otherActor )
				{
					handInfo( hand ).handTouches++;
				}
			}
		}

		if ( pair.status & PxPairFlag::eNOTIFY_TOUCH_LOST)
		{
			for ( Hand hand : BothHands )
			{
				if ( handInfo( hand ).handCollider == pair.triggerActor || handInfo( hand ).handCollider == pair.otherActor )
				{
					handInfo( hand ).handTouches--;
				}
			}
		}
	}

}

void XRSApp::onContact( const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 count )
{
	for ( uint32_t i = 0; i < count; i++ )
	{
		const PxContactPair& contact = pairs[ i ];
		if ( contact.events & PxPairFlag::eNOTIFY_TOUCH_FOUND )
		{
			for ( Hand hand : BothHands )
			{
				if ( handInfo( hand ).handCollider == contact.shapes[ 0 ]->getActor() 
					|| handInfo( hand ).handCollider == contact.shapes[ 1 ]->getActor() )
				{
					handInfo( hand ).handTouches++;
				}
			}
		}

		if ( contact.events & PxPairFlag::eNOTIFY_TOUCH_LOST )
		{
			for ( Hand hand : BothHands )
			{
				if ( handInfo( hand ).handCollider == contact.shapes[ 0 ]->getActor()
					|| handInfo( hand ).handCollider == contact.shapes[ 1 ]->getActor() )
				{
					handInfo( hand ).handTouches--;
				}
			}
		}
	}
}
