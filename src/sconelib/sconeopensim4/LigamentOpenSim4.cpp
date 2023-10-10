#include "LigamentOpenSim4.h"

#include "ModelOpenSim4.h"
#include "DofOpenSim4.h"
#include "simbody_tools.h"

namespace scone
{
	LigamentOpenSim4::LigamentOpenSim4( ModelOpenSim4& model, const OpenSim::Ligament& mus ) :
		Ligament( model ),
		m_Model( model ),
		m_osMus( mus )
	{
		InitJointsDofs();
	}

	LigamentOpenSim4::~LigamentOpenSim4()
	{}

	const Body& LigamentOpenSim4::GetOriginBody() const
	{
		auto& pps = m_osMus.getGeometryPath().getPathPointSet();
		return *FindByName( m_Model.GetBodies(), pps.get( 0 ).getBodyName() );
	}

	const Body& LigamentOpenSim4::GetInsertionBody() const
	{
		auto& pps = m_osMus.getGeometryPath().getPathPointSet();
		return *FindByName( m_Model.GetBodies(), pps.get( pps.getSize() - 1 ).getBodyName() );
	}

	const Model& LigamentOpenSim4::GetModel() const
	{
		return m_Model;
	}

	Real LigamentOpenSim4::GetMomentArm( const Dof& dof ) const
	{
		const auto& dof_sb = dynamic_cast<const DofOpenSim4&>( dof );
		return m_osMus.getGeometryPath().computeMomentArm( m_Model.GetTkState(), dof_sb.GetOsCoordinate() );
	}

	Real LigamentOpenSim4::GetPcsaForce() const
	{
		return m_osMus.get_pcsa_force();
	}

	Real LigamentOpenSim4::GetRestingLength() const
	{
		return m_osMus.getRestingLength();
	}

	Real LigamentOpenSim4::GetForce() const
	{
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Position );
		return m_osMus.getTension( m_Model.GetTkState() );
	}

	Real LigamentOpenSim4::GetNormalizedForce() const
	{
		return GetForce() / GetPcsaForce();
	}

	Real LigamentOpenSim4::GetLength() const
	{
		m_Model.GetOsimModel().getMultibodySystem().realize( m_Model.GetTkState(), SimTK::Stage::Position );
		return m_osMus.getLength( m_Model.GetTkState() );
	}

	Real LigamentOpenSim4::GetNormalizedLength() const
	{
		return GetLength() / GetRestingLength();
	}

	Real LigamentOpenSim4::GetVelocity() const
	{
		return 0.0;
	}

	Real LigamentOpenSim4::GetNormalizedVelocity() const
	{
		return GetVelocity() / GetRestingLength();
	}

	std::vector<Vec3> LigamentOpenSim4::GetLigamentPath() const
	{
		auto& pps = m_osMus.getGeometryPath().getCurrentPath( m_Model.GetTkState() );
		std::vector< Vec3 > points( pps.getSize() );
		for ( int i = 0; i < points.size(); ++i ) {
			const auto& mob = m_Model.GetOsimModel().getMultibodySystem().getMatterSubsystem().getMobilizedBody( pps[i]->getBody().getMobilizedBodyIndex() );
			auto world_pos = mob.getBodyTransform( m_Model.GetTkState() ) * pps[i]->getLocation( m_Model.GetTkState() );
			points[i] = from_osim( world_pos );
		}
		return points;
	}

	std::vector<std::pair<Body*, Vec3>> LigamentOpenSim4::GetLocalLigamentPath() const
	{
		auto& pps = m_osMus.getGeometryPath().getCurrentPath( m_Model.GetTkState() );
		std::vector< std::pair< Body*, Vec3 > > points;
		for ( int i = 0; i < pps.size(); ++i )
			points.emplace_back( FindByName( m_Model.GetBodies(), pps[i]->getBody().getName() ), from_osim( pps[i]->getLocation( m_Model.GetTkState() ) ) );
		return points;
	}

	const String& LigamentOpenSim4::GetName() const
	{
		return m_osMus.getName();
	}

}

