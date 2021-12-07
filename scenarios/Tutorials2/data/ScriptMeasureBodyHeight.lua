-- SCONE script for a high-jump measure.
-- See Tutorial 6a - Script - High Jump

function init( model, par, side )
	-- get the 'target_body' parameter from ScriptMeasure, or set to "pelvis"
	target_body = scone.target_body or "pelvis"
	
	-- find the actual body with the same name
	body = model:find_body( target_body )
	
	-- body height integral, used to compute average height
	body_H = 0
end

function update( model )
	-- get current vertical position and velocity
	local body_height = body:com_pos().y
	
	-- update best_height
	body_H = body_H + model:delta_time() * body_height
	
	-- stop simulation if com pos is below 0.5
	if model:com_pos().y < 0.5 then
		return true -- terminate the simulation
	else
		return false -- keep simulating
	end
end

function result( model )
	-- this is called at the end of the simulation
	-- fitness corresponds to average body height
	local fitness = body_H / model:max_duration()
	
	return fitness
end

function store_data( frame )
	-- store some values for analysis
	frame:set_value( "body_height", body:com_pos().y )
end
