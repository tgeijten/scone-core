import numpy as np
import time
import math
from sconetools import sconepy

# Run a simulation in which reflexes of a SCONE Gait Controller are modulated
def run_simulation(model, control_name, control_base, control_delta, max_time=3, min_com_height=0.3):
	# Get start time for simulation performance
	start_time = time.perf_counter()

	# Reset the model to the initial state
	# IMPORTANT: this does not reset the control parameters
	model.reset()

 	# Start the simulation, with time t ranging from 0 to max_time
	for t in np.arange(0, max_time, 0.01):
		# Modulate the reflex control parameter, every 0.01s
		model.set_control_parameter(control_name, control_base + math.sin(t/math.pi) * control_delta)

		# Advance the simulation to time t
		# This will automatically update the SCONE Gait Controller
		model.advance_simulation_to(t)

		# Abort the simulation if the model center of mass falls below min_com_height
		com_y = model.com_pos().y
		if com_y < min_com_height:
			break

	# Show performance
	duration = time.perf_counter() - start_time
	print(f"Simulation terminated at t={model.time():.3f} com_y={com_y:.4f} ({model.time() / duration:0.2f}x real-time)")

	# Write results to the SCONE results folder
	# The resulting .sto file can be openend directly in SCONE Studio for analysis
	if model.get_store_data():
		dirname = "sconepy_reflex_gait_" + model.name()
		filename = model.name() + f'_{control_delta}_{model.time():0.3f}'
		model.write_results(dirname, filename)
		print(f"Results written to {dirname}/{filename}", flush=True)

# Set the SCONE log level between 1-7 (lower is more logging)
sconepy.set_log_level(3)
print("SCONE Version", sconepy.version())

# Run a couple of simulations
if sconepy.is_supported("ModelHyfydy"):
	# Load a model with a .par file
	model = sconepy.load_model("data/H0918_gait_hfd.scone", "data/H0918GaitRS2Hfd4.par")
	model.set_store_data(True)
	simulation_duration = 10

	# Initialize the control parameter we wish to modulate and store its base value
	control_name = 'S11111.soleus.KF'
	control_base = model.get_control_parameter(control_name)

	# Run simulations with different control parameter modulation settings
	run_simulation(model, control_name, control_base, 0.01, simulation_duration)
	run_simulation(model, control_name, control_base, 0.03, simulation_duration)
	run_simulation(model, control_name, control_base, 0.08, simulation_duration)

	print("Please open the .sto files in SCONE Studio to see the results")
