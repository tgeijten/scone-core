-- SCONE script that simulates a device that generates an external moment at a specific condition.
-- See Tutorial 6b - Script - Balance Device
function init( model, par, side )
	treadmill = model:find_body("treadmill")
	speed = 1

	-- set speed of treadmill
	lin_vel = vec3:new( -speed, 0, 0 )
	treadmill:set_lin_vel(lin_vel)
end

function update( model )
	-- move back position of treadmill object
	local tp = treadmill:com_pos()
	if tp.x < -1 then
		tp.x = tp.x + 2
		treadmill:set_com_pos( tp )
	end

	-- return false to keep going
	return false;
end
