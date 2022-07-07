#include "ContactForceOpenSim3.h"

#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "ModelOpenSim3.h"
#include "scone/model/model_tools.h"
#include <variant>
#include "xo/geometry/quat.h"
#include "scone/core/Log.h"

namespace scone
{
	std::vector<std::string> make_string_list( const OpenSim::Property<std::string>& osList ) {
		std::vector<std::string> list;
		for ( int idx = 0; idx < osList.size(); ++idx )
			list.push_back( osList.getValue( idx ) );
		return list;
	}

	ContactForceOpenSim3::ContactForceOpenSim3( ModelOpenSim3& model, const OpenSim::Force& osForce ) :
		m_osForce( osForce ),
		m_Model( model ),
		m_LastNumDynamicsRealizations( -1 ),
		m_Force(),
		m_Moment(),
		m_Point(),
		m_PlaneLocation(),
		m_PlaneNormal()
	{
		std::vector<std::string> geom_names;
		if ( auto* hcf = dynamic_cast<const OpenSim::HuntCrossleyForce*>( &osForce ) )
		{
			auto hcfnc = const_cast<OpenSim::HuntCrossleyForce&>( *hcf ); // hack for OpenSim bug
			geom_names = make_string_list( hcfnc.getContactParametersSet().get( 0 ).getGeometry() );
		}
		else if ( auto* eff = dynamic_cast<const OpenSim::ElasticFoundationForce*>( &osForce ) )
		{
			auto effnc = const_cast<OpenSim::ElasticFoundationForce&>( *eff ); // hack for OpenSim bug
			geom_names = make_string_list( effnc.getContactParametersSet().get( 0 ).getGeometry() );
		}
		else SCONE_ERROR( "Unsupported contact force: " + osForce.getName() );

		for ( const auto& name : geom_names )
		{
			auto& cg = *FindByName( model.GetContactGeometries(), name );
			if ( auto p = std::get_if< xo::plane >( &cg.GetShape() ) )
			{
				// initialize plane normal / pos, needed for cop computation
				m_PlaneNormal = cg.GetOri() * Vec3( p->normal_ );
				m_PlaneLocation = cg.GetPos();
			}
			m_Geometries.push_back( &cg );
		}

		auto labels = m_osForce.getRecordLabels();
		m_Labels.reserve( labels.size() );
		for ( int i = 0; i < labels.size(); ++i )
			m_Labels.push_back( labels[ i ] );
		m_Values.resize( labels.size() );

		// attach contact force to bodies
		for ( auto& b : model.GetBodies() )
		{
			for ( auto& cg : m_Geometries )
				if ( &cg->GetBody() == b )
					dynamic_cast<BodyOpenSim3&>( *b ).AttachContactForce( this );
		}
	}

	ContactForceOpenSim3::~ContactForceOpenSim3()
	{}

	const String& ContactForceOpenSim3::GetName() const
	{
		return m_osForce.getName();
	}

	const Vec3& ContactForceOpenSim3::GetForce() const
	{
		UpdateForceValues();
		return m_Force;
	}

	const Vec3& ContactForceOpenSim3::GetMoment() const
	{
		UpdateForceValues();
		return m_Moment;
	}

	const Vec3& ContactForceOpenSim3::GetPoint() const
	{
		UpdateForceValues();
		return m_Point;
	}

	std::tuple<const Vec3&, const Vec3&, const Vec3&> ContactForceOpenSim3::GetForceMomentPoint() const
	{
		UpdateForceValues();
		return { m_Force, m_Moment, m_Point };
	}

	ForceValue ContactForceOpenSim3::GetForceValue() const
	{
		UpdateForceValues();
		return { m_Force, m_Point };
	}

	void ContactForceOpenSim3::UpdateForceValues() const
	{
		auto& osModel = m_Model.GetOsimModel();
		auto& tkState = m_Model.GetTkState();

		// realize the state and check the number of realizations
		// IMPORTANT: we use the getNumRealizationsOfThisStage() instead of time or step
		// because FixTkState() calls this multiple times before integration
		osModel.getMultibodySystem().realize( tkState, SimTK::Stage::Dynamics );
		int num_dyn = osModel.getMultibodySystem().getNumRealizationsOfThisStage( SimTK::Stage::Dynamics );

		// update m_ContactForceValues only if needed (performance)
		if ( m_LastNumDynamicsRealizations != num_dyn )
		{
			OpenSim::Array<double> forces = m_osForce.getRecordValues( tkState );
			for ( int i = 0; i < forces.size(); ++i )
				m_Values[ i ] = forces[ i ];
			m_LastNumDynamicsRealizations = num_dyn;

			m_Force.set( -m_Values[ 0 ], -m_Values[ 1 ], -m_Values[ 2 ] );
			m_Moment.set( -m_Values[ 3 ], -m_Values[ 4 ], -m_Values[ 5 ] );
			m_Point = GetPlaneCop( m_PlaneNormal, m_PlaneLocation, m_Force, m_Moment );
		}
	}
}
