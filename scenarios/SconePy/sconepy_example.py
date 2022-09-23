# This section is required to find the sconepy module and DLLs
# Change SCONE_BIN_DIR if SCONE was installed to a different folder
SCONE_BIN_DIR = 'C:/Program Files/SCONE/bin'
# SCONE_BIN_DIR = 'D:/Build/scone-studio/vc2019-x64/bin/Release'
import sys
sys.path.append(SCONE_BIN_DIR)

# Import the sconepy module (should be in the right path)
import sconepy
import numpy as np
import time

# Helper function that shows various model properties
def print_model_info(model):
	# iterate over bodies and print some properties
	# other available objects are: model.dofs(), model.actuators(), model.state()
	for bod in model.bodies():
		if bod.name().startswith('t'): # only items starting with 't' to avoid clutter
			print(f"body {bod.name()} mass={bod.mass():.3f} inertia={bod.inertia_diag()}")
	for mus in model.muscles():
		if mus.name().startswith('g'): # only items starting with 'g' to avoid clutter
			print(f"muscle {mus.name()} L={mus.fiber_length_norm():.3f} V={mus.fiber_velocity_norm():.3f} F={mus.force_norm():.3f}")

# Function that loads a model and runs a simulation
def run_simulation(model, store_data, random_seed, max_time=3, min_com_height=0.3):
	# reset the model to the initial state
	model.reset()
	model.set_store_data(store_data)

	# initialize the rng
	rng = np.random.default_rng(random_seed)

	# Initialize muscle activations to random values
	muscle_activations = 0.1 + 0.4 * rng.random((len(model.muscles())))
	model.init_muscle_activations(muscle_activations)

	# Tweak the initial pose of the model
	dof_positions = model.dof_position_array()
	dof_positions += 0.1 * rng.random(len(dof_positions)) - 0.05
	model.set_dof_positions(dof_positions)
	for d in model.dofs():
		if d.name() == 'pelvis_ty':
			d.set_pos(0.1 + d.pos()) # set the value of a

	# IMPORTANT: set the actual pose and equilibrates the muscles
	model.init_state_from_dofs()

 	# Start the simulation, with time t ranging from 0 to 5
	for t in np.arange(0, max_time, 0.01):

		# Set actuator_inputs based on muscle force, length and velocity
		mus_in = model.muscle_force_array()
		mus_in += model.muscle_fiber_length_array() - 1
		mus_in += 0.2 * model.muscle_fiber_velocity_array()
		model.set_actuator_inputs(mus_in)

		# Advance the simulation to time t
		# Internally, this performs as many simulations steps as required
		# The internal step size is variable, and determined by the 'accuracy'
		# setting in the .scone file
		model.advance_simulation_to(t)

		# Abort the simulation if the model center of mass falls below 0.3 meter
		com_y = model.com_pos().y
		if com_y < min_com_height:
			print(f'Aborting simulation at t={model.time():.3f} com_y={com_y:.4f}')
			break

	# Write results to the SCONE results folder
	# The resulting .sto file can be openend directly in SCONE Studio for analysis
	if store_data:
		dirname = 'sconepy_example_' + model.name()
		filename = model.name() + f'_{random_seed}_{model.time():0.3f}_{model.com_pos().y:0.3f}'
		model.write_results(dirname, filename)
		print(f'Results written to {dirname}/{filename}; please use SCONE Studio to replay the .sto file.', flush=True)


def measure_performance(model_file):
	# load the model
	start_time = time.perf_counter()
	model = sconepy.load_model(model_file)
	load_time = time.perf_counter() - start_time

	# run a couple of simulations
	random_seed = 1
	model_time = 0.0
	duration = 0.0
	start_time = time.perf_counter()
	while duration < 2.0:
		run_simulation(model, False, random_seed, max_time=2, min_com_height=-10)
		duration = time.perf_counter() - start_time
		random_seed += 1
		model_time += model.time()

	# show results
	real_time_factor = model_time / duration
	print(f'Loading {model_file} took {load_time*1000:.2f}ms - Simulating took {duration:.2f}s for {model_time:0.2f}s ({real_time_factor:0.2f}x real-time)', flush=True)


# Set the SCONE log level between 1-7 (lower is more logging)
sconepy.set_log_level(3)
print('SCONE Version', sconepy.version())

# change the datatype of returned arrays to 32-bit floats
sconepy.set_array_dtype_float32()

# Run a couple of simulations
# IMPORTANT: use store_data=False when measuring performance
store_data = True

# Create a SCONE model from a .scone file
model = sconepy.load_model('data/H0918_hyfydy.scone')
# model = sconepy.load_model( 'H1622_hyfydy.scone' )
# model = sconepy.load_model( 'H0918_osim3.scone' )
# model = sconepy.load_model( 'H0918_osim4.scone' )
print_model_info(model)
run_simulation(model, store_data, 1)
run_simulation(model, store_data, 2)
run_simulation(model, store_data, 1)

# measure_performance('data/H0918_hyfydy.scone')
# measure_performance( 'H0918_osim3.scone' )
# measure_performance( 'H0918_osim4.scone' )
