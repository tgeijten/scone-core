-- controller for monosynaptic length and force reflexes
function init( model, par, side )
	-- symmetric determines if the feedback gains are the same for left and right
	symmetric = tonumber(scone.symmetric or "1") > 0

	-- keep a list of offsets and slopes to compute the excitation
	L0 = {}
	KL = {}
	KF = {}
	
	-- keep a list of all actuators
	sensorL = {}
	sensorF = {}
	actuator = {}
	
	-- iterate over all muscles in the model
	for i = 1, model:muscle_count() do
		mus = model:muscle(i)

		-- get the muscle name without _l or _r postfix
		local name = string.sub(mus:name(), 1, -3)

		-- look up the neural delay for this muscle, defined in the Model
		local delay = model:find_one_way_neural_delay( name )
		scone.debug(mus:name() .. " has one-way neural delay " .. tostring(delay))

		-- create delayed sensors and actuators
		sensorL[i] = mus:create_delayed_length_sensor( delay )
		sensorF[i] = mus:create_delayed_force_sensor( delay )
		actuator[i] = mus:create_delayed_actuator( delay )

		-- create parameters for length and force feedback
		local parname = symmetric and name or mus:name()
		L0[i] = par:create_from_mean_std( parname .. ".L0", 0.9, 0.01, 0.1, 1.5 )
		KL[i] = par:create_from_mean_std( parname .. ".KL", 0.5, 0.01, 0, 4 )
		KF[i] = par:create_from_mean_std( parname .. ".KF", 0.5, 0.01, 0, 4 )
	end
end

function update( model )
	-- iterate over all delayed actuators
	for i = 1, #actuator do
		-- get delayed muscle length L and force F
		local L = sensorL[i]:value() - L0[i]
		local F = sensorF[i]:value()

		-- apply delayed excitation using feedback gains KL and KF
		local excitation =  KL[i] * L + KF[i] * F
		actuator[i]:add_input( excitation )
	end

	return false
end
