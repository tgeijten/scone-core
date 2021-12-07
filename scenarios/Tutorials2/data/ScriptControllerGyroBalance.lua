-- SCONE script that simulates a device that generates an external moment at a specific condition.
-- See Tutorial 6b - Script - Gyroscopic Balance Device

function init( model, par )
	-- get the 'target_body' parameter from ScriptController, or set to "pelvis"
	target_body = model:find_body( scone.target_body or "pelvis" )
	
	-- get 'min_deg' and 'max_deg' parameters from ScriptController (required!)
	if ( scone.min_deg and scone.min_deg ) then
		body_min = par:create_from_string( "min_deg", scone.min_deg )
		body_max = par:create_from_string( "max_deg", scone.max_deg )
	else
		-- produce an error and abort the simulation
		error( "Must set min_deg and max_deg parameters!" )
	end
	
	-- get 'gyro_moment' and 'gyro_duration' parameters from ScriptController (not required!)
	pos_moment_gain = scone.pos_moment_gain and par:create_from_string( "pos_moment_gain", scone.pos_moment_gain ) or 50
	neg_moment_gain = scone.neg_moment_gain and par:create_from_string( "neg_moment_gain", scone.neg_moment_gain ) or 50
	gyro_duration = scone.duration and par:create_from_string( "gyro_duration", scone.duration ) or 0.2
	
	-- initialize global variables that keep track of the gyro state
	gyro_start = 0
	gyro_moment = 0
end

function update( model )
	local t = model:time()
	
	-- check if the gyro is currently active
	if gyro_moment ~= 0 then
		-- check if the gyro should deactivate
		if t > gyro_start + gyro_duration then
			-- add opposite external moment to deactivate gyro
			target_body:add_external_moment( 0, 0, -gyro_moment )
			gyro_moment = 0
			gyro_start = 0
			-- print a message to the scone messages window
			scone.debug( "gyro deactivated at " .. t )
		end
		-- gyro is not active, check if we activate it
	elseif gyro_start == 0 then -- gyro was not activated before
		-- get the current body orientation in degrees and check with body_min / body_max
		local ori = 57.3 * target_body:ang_pos().z
		local ang_vel = target_body:ang_vel().z
		if ori < body_min and ang_vel < 0 then
			gyro_moment = -ang_vel * pos_moment_gain
		elseif ori > body_max and ang_vel > 0 then
			gyro_moment = -ang_vel * neg_moment_gain
		end
		
		-- check if gyro_moment was set
		if gyro_moment ~= 0 then
			gyro_start = t
			target_body:add_external_moment( 0, 0, gyro_moment )
			
			-- print a message to the scone messages window
			scone.debug( "gyro activated at t=" .. t .. " ori=" .. ori .. " moment=" .. gyro_moment )
		end
	end
	
	-- return false to keep going
	return false;
end

function store_data( frame )
	-- store some values for analysis
	frame:set_value( "gyro_moment", gyro_moment )
end
