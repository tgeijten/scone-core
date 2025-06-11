#include "MuscleGroup.h"
#include "scone/core/IncludeExcludePattern.h"
#include "Model.h"
#include "xo/container/container_tools.h"
#include "xo/geometry/mirror_tools.h"

namespace scone
{
	MuscleGroup::MuscleGroup( const Model& m, const PropNode& pn, bool mirror ) :
		Muscle( m ),
		INIT_MEMBER_REQUIRED( pn, name_ ),
		total_weight_( 0 )
	{
		auto pattern_match = pn.get<IncludeExcludePattern>();
		if ( mirror ) {
			xo::mirror( name_ );
			pattern_match.mirror_patterns();
		}
		for ( auto* mus : m.GetMuscles() ) {
			if ( pattern_match( mus->GetName() ) ) {
				auto w = mus->GetMaxIsometricForce();
				muscles_.emplace_back( w, mus );
				total_weight_ += w;
				xo::merge( m_Joints, mus->GetJoints() );
				xo::merge( m_Dofs, mus->GetDofs() );
			}
		}
	}

	const String& MuscleGroup::GetName() const
	{
		return name_;
	}

	void MuscleGroup::AddInput( double v )
	{
		Actuator::AddInput( v );
		for ( auto&& m : muscles_ )
			m.second->AddInput( v );
	}

	const Body& MuscleGroup::GetOriginBody() const
	{
		return GetMainMuscle().GetOriginBody();
	}

	const Body& MuscleGroup::GetInsertionBody() const
	{
		return GetMainMuscle().GetInsertionBody();
	}

	const Model& MuscleGroup::GetModel() const
	{
		return GetMainMuscle().GetModel();
	}

	Real MuscleGroup::GetMomentArm( const Dof& dof ) const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetMomentArm( dof ); } );
	}

	Vec3 MuscleGroup::GetMomentArm3D( const Joint& joint ) const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetMomentArm3D( joint ); }, Vec3::zero() );
	}

	Real MuscleGroup::GetMaxIsometricForce() const
	{
		return GetSum( [&]( const Muscle& m ) { return m.GetMaxIsometricForce(); } );
	}

	Real MuscleGroup::GetOptimalFiberLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetOptimalFiberLength(); } );
	}

	Real MuscleGroup::GetTendonSlackLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetTendonSlackLength(); } );
	}

	Real MuscleGroup::GetPennationAngleAtOptimal() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetPennationAngleAtOptimal(); } );
	}

	Real MuscleGroup::GetForce() const
	{
		return GetSum( [&]( const Muscle& m ) { return m.GetForce(); } );
	}

	Real MuscleGroup::GetNormalizedForce() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetNormalizedForce(); } );
	}

	Real MuscleGroup::GetLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetLength(); } );
	}

	Real MuscleGroup::GetVelocity() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetVelocity(); } );
	}

	Real MuscleGroup::GetFiberForce() const
	{
		return GetSum( [&]( const Muscle& m ) { return m.GetFiberForce(); } );
	}

	Real MuscleGroup::GetActiveFiberForce() const
	{
		return GetSum( [&]( const Muscle& m ) { return m.GetActiveFiberForce(); } );
	}

	Real MuscleGroup::GetPassiveFiberForce() const
	{
		return GetSum( [&]( const Muscle& m ) { return m.GetPassiveFiberForce(); } );
	}

	Real MuscleGroup::GetFiberLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetFiberLength(); } );
	}

	Real MuscleGroup::GetNormalizedFiberLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetNormalizedFiberLength(); } );
	}

	Real MuscleGroup::GetCosPennationAngle() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetCosPennationAngle(); } );
	}

	Real MuscleGroup::GetFiberVelocity() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetFiberVelocity(); } );
	}

	Real MuscleGroup::GetNormalizedFiberVelocity() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetNormalizedFiberVelocity(); } );
	}

	Real MuscleGroup::GetTendonLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetTendonLength(); } );
	}

	Real MuscleGroup::GetNormalizedTendonLength() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetNormalizedTendonLength(); } );
	}

	Real MuscleGroup::GetActiveForceLengthMultipler() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetActiveForceLengthMultipler(); } );
	}

	Real MuscleGroup::GetForceVelocityMultipler() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetForceVelocityMultipler(); } );
	}

	Real MuscleGroup::GetMaxContractionVelocity() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetMaxContractionVelocity(); } );
	}

	std::vector<Vec3> MuscleGroup::GetMusclePath() const
	{
		return std::vector<Vec3>();
	}

	std::vector< std::pair< Body*, scone::Vec3 > > MuscleGroup::GetLocalMusclePath() const
	{
		return std::vector< std::pair< Body*, scone::Vec3 > >();
	}

	Real MuscleGroup::GetActivation() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetActivation(); } );
	}

	Real MuscleGroup::GetExcitation() const
	{
		return GetAverage( [&]( const Muscle& m ) { return m.GetExcitation(); } );
	}

	void MuscleGroup::SetExcitation( Real u )
	{
		SCONE_ERROR( "Unexpected call to MuscleGroup::SetExcitation()" );
	}

	void MuscleGroup::InitializeActivation( Real u )
	{
		SCONE_ERROR( "Unexpected call to MuscleGroup::InitializeActivation()" );
	}

	PropNode MuscleGroup::GetInfo() const
	{
		PropNode pn = Muscle::GetInfo();
		auto& mus_pn = pn["muscles"];
		for ( auto m : muscles_ )
			mus_pn[m.second->GetName()] = m.first;
		return pn;
	}
}

