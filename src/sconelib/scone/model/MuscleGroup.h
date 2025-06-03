#pragma once

#include "Muscle.h"

namespace scone
{
	class SCONE_API MuscleGroup : public Muscle
	{
	public:
		MuscleGroup( const Model& m, const PropNode& pn );
		virtual ~MuscleGroup() = default;

		// HasName overrides
		virtual const String& GetName() const override;

		// Actuator overrides
		virtual void AddInput( double v ) override;

		// Muscle overrides
		virtual const Body& GetOriginBody() const override;
		virtual const Body& GetInsertionBody() const override;
		virtual const Model& GetModel() const override;
		virtual Real GetMomentArm( const Dof& dof ) const override;
		virtual Vec3 GetMomentArm3D( const Joint& joint ) const override;
		virtual Real GetMaxIsometricForce() const override;
		virtual Real GetOptimalFiberLength() const override;
		virtual Real GetTendonSlackLength() const override;
		virtual Real GetPennationAngleAtOptimal() const override;
		virtual Real GetForce() const override;
		virtual Real GetNormalizedForce() const override;
		virtual Real GetLength() const override;
		virtual Real GetVelocity() const override;
		virtual Real GetFiberForce() const override;
		virtual Real GetActiveFiberForce() const override;
		virtual Real GetPassiveFiberForce() const override;
		virtual Real GetFiberLength() const override;
		virtual Real GetNormalizedFiberLength() const override;
		virtual Real GetCosPennationAngle() const override;
		virtual Real GetFiberVelocity() const override;
		virtual Real GetNormalizedFiberVelocity() const override;
		virtual Real GetTendonLength() const override;
		virtual Real GetNormalizedTendonLength() const override;
		virtual Real GetActiveForceLengthMultipler() const override;
		virtual Real GetForceVelocityMultipler() const override;
		virtual Real GetMaxContractionVelocity() const override;
		virtual std::vector<Vec3> GetMusclePath() const override;
		virtual std::vector< std::pair< Body*, Vec3 > > GetLocalMusclePath() const override;
		virtual Real GetActivation() const override;
		virtual Real GetExcitation() const override;
		virtual void SetExcitation( Real u ) override;
		virtual void InitializeActivation( Real u ) override;
		virtual PropNode GetInfo() const override;

		// non-virtual members
		std::vector<std::pair<Real, Muscle*>> const GetMuscles() { return muscles_; }

	protected:
		String name_;
		std::vector<std::pair<Real, Muscle*>> muscles_;
		Real total_weight_;

		template< typename T = Real, typename Func >
		T GetAverage( Func f, T value = T() ) const {
			for ( auto&& m : muscles_ )
				value += m.first * f( *m.second );
			return value / total_weight_;
		}

		template< typename T = Real, typename Func >
		T GetSum( Func f, T value = T() ) const {
			for ( auto&& mus : muscles_ )
				value += f( *mus.second );
			return value;
		}

		Muscle& GetMainMuscle() const { return *muscles_.front().second; }
	};
}
