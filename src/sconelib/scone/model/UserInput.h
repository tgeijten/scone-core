#pragma once

#include "scone/core/types.h"
#include "scone/core/HasName.h"
#include "scone/core/Range.h"

namespace scone
{
	class SCONE_API UserInput
	{
	public:
		UserInput() = default;
		virtual ~UserInput() = default;

		virtual const String& GetName() const = 0;
		virtual const String& GetLabel() const = 0;
		virtual const String& GetGroupName() const = 0;
		virtual Real GetValue() const = 0;
		virtual Range<Real> GetRange() const = 0;

		virtual void SetValue( Real v ) = 0;
	};
}
