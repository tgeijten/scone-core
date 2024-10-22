import numpy as np
import time
from sconetools import sconepy

# Set the SCONE log level to 3
sconepy.set_log_level(3)

# Show which SCONE version we are using
print("SCONE Version", sconepy.version())

# Load an example scenario
scenario = sconepy.load_scenario("../Examples/Jump2D - H0918 - Hyfydy.scone");

# Start multiple optimizations with different settings
num_optimizers = 6
opts = []
for i in range(0, num_optimizers):
	# Change some settings based on i
	scenario.set("CmaOptimizer.random_seed", str(i+1))
	scenario.set("CmaOptimizer.SimulationObjective.FeedForwardController.symmetric", str(i%2))

	# Start the optimization as background task
	opts.append(scenario.start_optimization())

# Wait for all optimizations to finish
num_finished = 0
while num_finished < num_optimizers:
	num_finished = 0
	time.sleep(1)
	# Iterate over all optimizers
	for i in range(0, num_optimizers):
		# Create a status string
		str = f"Optimization {i}: step={opts[i].current_step()} fitness={opts[i].fitness():.2f}";

		if opts[i].finished():
			# This optimization is finished, add to status string
			str += " FINISHED"
			num_finished += 1
		elif opts[i].current_step() >= 1000:
			# Optional: terminal optimizations after 1000 steps
			str += " TERMINATING"
			opts[i].terminate()

		# Print status 
		print(str, flush=True)

# At this point, all optimizations have finished
for i in range(0, num_optimizers):
	print(f"Finished optimization {i}: steps={opts[i].current_step()} fitness={opts[i].fitness():.2f}", flush=True)