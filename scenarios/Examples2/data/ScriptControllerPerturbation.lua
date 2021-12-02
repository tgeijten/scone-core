-- SCONE script that simulates a device that generates an external moment at a specific condition.
-- See Tutorial 6b - Script - Balance Device
function init( model, par, side )
	start_time = tonumber( scone.start_time ) or 3
	duration = tonumber( scone.duration ) or 0.1
	interval = tonumber( scone.interval ) or 1
	force = tonumber( scone.force ) or 100
	current_force_x = 0.0
	t0 = 0.0
	body_index = 2
end

function update( model )
	local t = model:time()
	
	if t < start_time then
		return false
	end
	
	
	if current_force_x == 0 and t - t0 > interval then
		-- start new perurbation
		current_force_x = force
		local body = model:body( body_index )
		body:add_external_force( current_force_x, 0, 0 )
		t0 = t
	end
	
	if current_force_x ~= 0 and t - t0 > duration then
		-- stop perturbation
		local body = model:body( body_index )
		body:add_external_force( -current_force_x, 0, 0 )
		current_force_x = 0
		
		-- rotate body_index, starting at 2
		if body_index < model:body_count() then
			body_index = body_index + 1
		else
			body_index = 2
		end
	end
	
	-- return false to keep going
	return false;
end
