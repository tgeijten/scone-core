const char* scone_settings_schema = R"str(
folders {
	scenarios { type = folder label = "Scenario folder" default = "" description = "Default location for SCONE scenarios" }
	results { type = folder label = "Results folder" default = "" description = "Default location for optimization results" }
	geometry { type = folder label = "Geometry folder" default = "" description = "Default location for model geometry" }
	geometry_extra { type = folder label = "Additional geometry" default = "" description = "Additional locations to look for model geometry" }
}

data {
	frequency { type = float label = "Data output frequency" default = 100 range = 10..1000000 }
	body { type = bool label = "Output body position and orientation" default = 1 }
	joint { type = bool label = "Output joint reation force" default = 1 }
	actuator { type = bool label = "Output actuator input" default = 1 }
	muscle { type = bool label = "Output muscle properties" default = 1 }
	muscle_detail { type = bool label = "Output detailed muscle properties" default = 0 }
	muscle_dof { type = bool label = "Output muscle moments and powers" default = 0 }
	grf { type = bool label = "Output ground reaction force" default = 1 }
	contact { type = bool label = "Output contact forces and moments" default = 0 }
	power { type = bool label = "Output system powers" default = 0 }
	sensor { type = bool label = "Output sensor data" default = 1 }
	controller { type = bool label = "Output controller data" default = 1 }
	measure { type = bool label = "Output measure data" default = 1 }
	simulation { type = bool label = "Output simulation statisticts" default = 0 }
	debug { type = bool label = "Output debug data" default = 0 }
	keep_all_frames { type = bool label = "Keep all data frames for analysis" default = 0 }
}

results {
	controller { type = bool label = "Output Controller and Measure results" default = 0 }
	extract_channels { type = bool label = "Extract specific channels to separate file" default = 0 }
	extract_channel_names { type = string label = "Channels to extract to separate file" default = "*.activation;*.excitation" }
}

optimizer {
	evaluator { type = number label = "Evaluate sync=0, batch=1, async=2, pool=3" default = 3 }
	max_threads { type = number label = "Max optimization threads (0=hardware)" default = 0 }
	thread_priority{ type = number label = "thread priority: 0-6 (default=2)" default = 2 }
	output_fitness_history { type = bool default = 1 label = "Output fitness history to history.txt" }
	output_par_history { type = bool default = 0 label = "Output parameter history to history_par.txt" }
	output_individual_search_points { type = bool default = 0 label = "Output .par files for all search points (use for debugging only)" }
}

hyfydy{
	enabled { type = boolean label = "Enable Hyfydy" default = 0 }
	license { type = string label = "Hyfydy license key" default = "" }
	license_agreement_accepted_version { type = int label = "Accepted Hyfydy license agreement version" default = 0 }
}
)str";
