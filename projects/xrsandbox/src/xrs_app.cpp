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


typedef std::map< std::string, uint32_t > XrExtensionMap;

struct WorldObject
{
	float4x4 objectToWorld;
	std::unique_ptr<GLTF::Model> model;
};

class XRSApp: public XrAppBase
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

		return true;
	}


	virtual void Render() override;
	virtual void Update( double currTime, double elapsedTime, XrTime displayTime ) override;
	virtual bool PreSession() override;
	virtual bool PostSession() override;
	virtual std::vector<std::string> GetDesiredExtensions();

	virtual bool RenderEye( int eye ) override;
	virtual void UpdateEyeTransforms( float4x4 eyeToProj, float4x4 stageToEye, XrView& view ) override;
	void UpdateHandPoses( XrHandTrackerEXT handTracker, GLTF::Model* model, XrTime displayTime );
	
	WorldObject* SpawnObject( const float4x4& objectToWorld, const std::string& modelPath );
private:
	RefCntAutoPtr<IPipelineState>		 m_pPSO;
	RefCntAutoPtr<IShaderResourceBinding> m_pSRB;
	float4x4							  m_ViewToProj;
	float4x4							m_handToWorld[ 2 ];
	bool								m_handToWorldValid[ 2 ] = { false, false };

	std::unique_ptr< XRDE::ActionSet > m_handActionSet;
	XRDE::Action * m_handAction;
	XRDE::Action * m_spawnAction;
	XRDE::Action * m_grabAction;
	XRDE::Action * m_hapticAction;

	std::unique_ptr<GLTF::Model> m_leftHandModel;
	std::unique_ptr<GLTF::Model> m_rightHandModel;

	std::vector< std::unique_ptr<WorldObject> > m_worldObjects;
	bool m_spawnObject[ 2 ] = { false, false };
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

using namespace XRDE;

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

	SetPbrEnvironmentMap( "textures/papermill.ktx" );

	m_leftHandModel = LoadGltfModel( "models/valve_hand_models/left_hand.glb" );
	m_rightHandModel = LoadGltfModel( "models/valve_hand_models/right_hand.glb" );

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
	for ( int cube = 0; cube < 2; cube++ )
	{
		if ( !m_handToWorldValid[ cube ] )
			continue;

		GLTF_PBR_Renderer::RenderInfo renderInfo;
		renderInfo.ModelTransform = float4x4::Identity();// m_handCubeToWorld[ cube ];

		if ( cube == 0 )
		{
			m_gltfRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *m_leftHandModel, renderInfo,
				nullptr, &m_CacheBindings );
		}
		else
		{
			m_gltfRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *m_rightHandModel, renderInfo,
				nullptr, &m_CacheBindings );
		}
	}

	for ( auto& worldObject : m_worldObjects )
	{
		GLTF_PBR_Renderer::RenderInfo renderInfo;
		renderInfo.ModelTransform = worldObject->objectToWorld;
		m_gltfRenderer->Render( m_pGraphicsBinding->GetImmediateContext(), *worldObject->model, renderInfo, nullptr, &m_CacheBindings );
	}

	return true;
}


// Render a frame
void XRSApp::Render()
{
}

enum class Hand
{
	Left = 0,
	Right = 1,
};

static const Hand BothHands[] = { Hand::Left, Hand::Right };

XrPath HandPath( Hand hand )
{
	switch ( hand )
	{
	case Hand::Left: return Paths().userHandLeft;
	case Hand::Right: return Paths().userHandRight;
	}
}

void XRSApp::Update( double CurrTime, double ElapsedTime, XrTime displayTime )
{
	// read input
	XrActiveActionSet activeActionSets[] =
	{
		{ m_handActionSet->Handle(), Paths().userHandLeft },
		{ m_handActionSet->Handle(), Paths().userHandRight },
	};
	XrActionsSyncInfo syncInfo = { XR_TYPE_ACTIONS_SYNC_INFO };
	syncInfo.activeActionSets = activeActionSets;
	syncInfo.countActiveActionSets = sizeof( activeActionSets ) / sizeof( activeActionSets[ 0 ] );
	xrSyncActions( m_session, &syncInfo );

	XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
	if( XR_SUCCEEDED( m_handAction->LocateSpace( m_stageSpace, displayTime, Paths().userHandLeft, &spaceLocation ) ) 
		&& ( spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT ) != 0 )
	{
		m_handToWorld[0] = matrixFromPose( spaceLocation.pose );
		m_handToWorldValid[ 0 ] = true;
	}
	else
	{
		m_handToWorldValid[ 0 ] = false;
	}
	if ( XR_SUCCEEDED( m_handAction->LocateSpace( m_stageSpace, displayTime, Paths().userHandRight, &spaceLocation ) )
		&& ( spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT ) != 0 )
	{
		m_handToWorld[ 1 ] = matrixFromPose( spaceLocation.pose );
		m_handToWorldValid[ 1 ] = true;
	}
	else
	{
		m_handToWorldValid[ 1 ] = false;
	}

	for ( Hand hand : BothHands )
	{
		int i = static_cast<int>( hand );
		bool oldSpawn = m_spawnObject[ i ];
		m_spawnObject[ i ] = m_spawnAction->GetBooleanState( m_session, HandPath( hand ) );

		if ( !oldSpawn && m_spawnObject[ i ] && m_handToWorldValid[ i ] )
		{
			m_hapticAction->ApplyHapticFeedback( m_session, Paths().userHandRight, 0, 20, 1 );
			SpawnObject( m_handToWorld[ i ], "models/gear.glb" );
		}
	}

	UpdateHandPoses( m_handTrackers[ 0 ], m_leftHandModel.get(), displayTime );
	UpdateHandPoses( m_handTrackers[ 1 ], m_rightHandModel.get(), displayTime );
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


void XRSApp::UpdateHandPoses( XrHandTrackerEXT handTracker, GLTF::Model* model, XrTime displayTime )
{
	if ( !m_enableHandTrackers )
		return;

	XrHandJointsLocateInfoEXT locateInfo = { XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT };
	locateInfo.time = displayTime;
	locateInfo.baseSpace = m_stageSpace;

	XrHandJointLocationEXT jointLocations[ XR_HAND_JOINT_COUNT_EXT ];
	XrHandJointLocationsEXT locations = { XR_TYPE_HAND_JOINT_LOCATIONS_EXT };
	locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
	locations.jointLocations = jointLocations;
	XrResult res = m_xrLocateHandJointsEXT( handTracker, &locateInfo, &locations );
	if ( XR_FAILED( res ) )
		return;

	if ( !locations.isActive )
		return;

	float4x4 jointsToParent[ XR_HAND_JOINT_COUNT_EXT ];
	float4x4 stageToJoint[ XR_HAND_JOINT_COUNT_EXT ];

	// pre-load the wrist because the palm is out of order and earlier in the enum
	jointsToParent[ XR_HAND_JOINT_WRIST_EXT ] = matrixFromPose( jointLocations[ XR_HAND_JOINT_WRIST_EXT ].pose );
	stageToJoint[ XR_HAND_JOINT_WRIST_EXT ] = jointsToParent[ XR_HAND_JOINT_WRIST_EXT ].Inverse();
	for ( uint32_t jointIndex = 0; jointIndex < XR_HAND_JOINT_COUNT_EXT; jointIndex++ )
	{
		if ( jointIndex == XR_HAND_JOINT_WRIST_EXT )
			continue;

		float4x4 jointToStage = matrixFromPose( jointLocations[ jointIndex ].pose );
		//float4x4 jointToStage = Diligent::float4x4::Translation( vectorFromXrVector( jointLocations[ jointIndex ].pose.position ) );
		stageToJoint[ jointIndex ] = jointToStage.Inverse();

		XrHandJointEXT parentJoint = GetParentJoint( ( XrHandJointEXT)jointIndex );
		if ( parentJoint == jointIndex )
		{
			// this joint has no parent, so its parent is the stage
			jointsToParent[ jointIndex ] = jointToStage;
		}
		else
		{
			jointsToParent[ jointIndex ] = jointToStage * stageToJoint[ parentJoint ];
		}
	}

	for ( auto& skin : model->Skins )
	{
		skin->Joints[ 0 ]->Translation = vectorFromXrVector( jointLocations[ 0 ].pose.position );
		skin->Joints[ 0 ]->Rotation = quaternionFromXrQuaternion( jointLocations[ 0 ].pose.orientation);
		for ( uint32_t handJoint = 0; handJoint < 26; handJoint++ )
		{
			//if ( handJoint > XR_HAND_JOINT_THUMB_TIP_EXT )
			//	break;

			uint32_t jointIndex = JointIndexFromHandJoint( (XrHandJointEXT)handJoint );

			GLTF::Node* node = skin->Joints[ jointIndex ];
			node->Matrix = jointsToParent[ handJoint ];
			node->Rotation = Quaternion( 0, 0, 0, 1.f );
			node->Translation = { 0, 0, 0 };
			node->Scale = { 1.f, 1.f, 1.f };
		}
	}

	for ( auto& root_node : model->Nodes )
	{
		root_node->UpdateTransforms();
	}
}

WorldObject* XRSApp::SpawnObject( const float4x4& objectToWorld, const std::string& modelPath )
{
	auto worldObject = std::make_unique<WorldObject>();
	worldObject->model = LoadGltfModel( modelPath );
	worldObject->objectToWorld = objectToWorld;
	WorldObject* p = worldObject.get();
	m_worldObjects.push_back( std::move( worldObject ) );
	return p;
}

