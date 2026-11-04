# The EEG decoder's verdict for one tagged hazard, from one analysis window.
builtin_interfaces/Time stamp
uint32 hazard_id
float32 frequency
float32 score        # response at the frequency / response at frequencies not being shown (1 = noise level)
bool present         # score above the threshold (for the required number of windows)
bool signal_ok       # false if the EEG stream is missing or the Oz contact is poor
