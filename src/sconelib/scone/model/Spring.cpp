#include "Spring.h"

#include "Body.h"
#include "Model.h"
#include "xo/container/container_tools.h"
#include "scone/core/Log.h"

namespace scone
{
	Vec3 Spring::GetParentPos() const
	{
		return GetParentBody().GetPosOfPointOnBody( GetPosInParent() );
	}

	Vec3 Spring::GetChildPos() const
	{
		return GetChildBody().GetPosOfPointOnBody( GetPosInChild() );
	}

	bool Spring::IsActive() const
	{
		return !( GetParentBody().IsStatic() && GetChildBody().IsStatic() );
	}

	void Spring::StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame[GetName() + ".length"] = GetLength();
		frame[GetName() + ".velocity"] = GetVelocity();
		frame[GetName() + ".force"] = GetForce();
	}

	void Spring::StoreStateData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const
	{
		frame[GetName() + ".parent_index"] = static_cast<Real>( GetParentBody().GetIndex() );
		frame.SetVec3( GetName() + ".parent", GetPosInParent() );
		frame[GetName() + ".child_index"] = static_cast<Real>( GetChildBody().GetIndex() );
		frame.SetVec3( GetName() + ".child", GetPosInChild() );
	}

	void Spring::SetStateFromData( const Storage<Real>::Frame& f )
	{
		auto& model = GetParentBody().GetModel();
		auto pidx = size_t( f[GetName() + ".parent_index"] );
		Vec3 ppos = f.GetVec3( GetName() + ".parent" );
		auto cidx = size_t( f[GetName() + ".child_index"] );
		Vec3 cpos = f.GetVec3( GetName() + ".child" );
		SCONE_ASSERT( pidx < model.GetBodies().size() && cidx < model.GetBodies().size() );
		SetParent( *model.GetBodies()[pidx], ppos );
		SetChild( *model.GetBodies()[cidx], cpos );
	}
}
