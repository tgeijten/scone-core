function init( model, par, side, cont )
	-- show all available control parameters
	-- WARNING: this can be slow, so don't use this in update()
	controls = cont:get_control_parameter_names()
	for i,v in ipairs(controls) do
		scone.debug(v .. " = " .. cont:get_control_parameter(v))
	end

	-- get initial values
	sol_kf = cont:get_control_parameter("S11111.soleus.KF")
	gas_kf = cont:get_control_parameter("S11111.gastroc.KF")
	ili_kl = cont:get_control_parameter("S11100.iliopsoas.KL")

	-- get amplification settings
	kf_inc = scone.kf_increase or 0.0;
	kl_inc = scone.kl_increase or 0.0;
end

function update( model, time, cont )
	-- set gradually increasing feedback gains
	local t = time / model:max_duration()
	cont:set_control_parameter("S11111.soleus.KF", t * kf_inc + sol_kf)
	cont:set_control_parameter("S11111.gastroc.KF", t * kf_inc + gas_kf)
	cont:set_control_parameter("S11100.iliopsoas.KL", t * kl_inc + ili_kl)

	return false
end
