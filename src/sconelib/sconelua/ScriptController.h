#pragma once

#include "platform.h"
#include "scone/controllers/CompositeController.h"
#include "scone/core/system_tools.h"

#include <functional>
#include <vector>

namespace scone
{
	using ArrayOfFiles = std::vector<path>;

	/// Controller defined through a Lua script.
	/** Runs the script defined in the file defined by ''script_file'', which is relative to the folder of the scone scenario:
	\verbatim
	ScriptController {
		script_file = "data/script_file.lua"
		external_files = [ "data/my_included_file.lua" ]
	}
	\endverbatim

	Example of a Lua controller script:
	\verbatim
	function init( model, par, side )
		-- This function is called at the start of the simulation
		-- 'model' can be used to initialize the desired actuators (see LuaModel)
		-- 'par' can be used to define parameters for optimization (see LuaPar)
		-- 'side' denotes if the controller is for a specific side (-1 = left, 0 = unspecified, 1 = right)
	end

	function update( model, time, controller )
		-- This function is called at each simulation timestep
		-- Use it to update the actuator inputs
		-- 'model' can be used to update the desired actuators (see LuaModel)
		-- 'time' is the time elapsed since this controller was activated
		-- 'controller' can be used to enable/disable child controllers of this ScriptController (see LuaController)
		return false -- change to 'return true' to terminate the simulation early
	end

	function store_data( current_frame )
		-- This function is called at each simulation timestep
		-- 'current_frame' can be used to store values for analysis (see LuaFrame)
	end
	\endverbatim
	See also LuaModel, LuaBody, LuaDof, LuaActuator, LuaMuscle, LuaFrame. See Tutorial 6a and 6b for more information.
	*/
	class SCONE_LUA_API ScriptController : public CompositeController
	{
	public:
		ScriptController( const PropNode& props, Params& par, Model& model, const Location& loc );
		virtual ~ScriptController();

		/// filename of the Lua script, path is relative to the .scone file
		path script_file;

		/// Array of files used by the Lua script; files included by 'require' should be added here as ''external_files = [ file1, file2 ]''
		ArrayOfFiles external_files;

		virtual void StoreData( Storage<Real>::Frame& frame, const StoreDataFlags& flags ) const override;

		int TrySetControlParameter( const String& name, Real value ) override;
		xo::optional<Real> TryGetControlParameter( const String& name ) override;
		std::vector<String> GetControlParameters() const override;
		bool CreateControlParameter( const String& name, Real value );

	protected:
		virtual bool ComputeControls( Model& model, double timestamp ) override;
		virtual String GetClassSignature() const override;

		xo::flat_map<String, Real> control_parameters_;

		u_ptr< class lua_script > script_;
		std::function<void( struct LuaModel*, struct LuaParams*, double, struct LuaController* )> init_;
		std::function<bool( struct LuaModel*, double, struct LuaController* )> update_;
		std::function<double( struct LuaFrame* )> store_;
	};
}
