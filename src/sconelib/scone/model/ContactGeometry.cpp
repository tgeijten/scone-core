#include "ContactGeometry.h"

namespace scone
{

	ContactGeometry::ContactGeometry( const String& name, const Body& b, const xo::shape& s, const Vec3& p, const Quat& q ) :
		m_Name( name ),
		m_Body( b ),
		m_Shape( s ),
		m_Pos( p ),
		m_Ori( q ),
		m_FileName()
	{}

	ContactGeometry::ContactGeometry( const String& name, const Body& b, const xo::path& f, const Vec3& p, const Quat& q ) :
		m_Name( name ),
		m_Body( b ),
		m_Shape(),
		m_Pos( p ),
		m_Ori( q ),
		m_FileName( f )
	{}

	PropNode ContactGeometry::GetInfo() const
	{
		PropNode pn;
		pn["name"] = GetName();
		pn.append( to_prop_node( GetShape() ) );
		pn["body"] = GetBody().GetName();
		pn["pos"] = GetPos();
		pn["ori"] = Vec3( xo::vec3degd( xo::euler_xyz_from_quat( GetOri() ) ) );

		return pn;
	}
}
