# Decoder status at ~4 Hz (/bci/status)
builtin_interfaces/Time stamp
int32[] marker_ids
string[] labels
float32[] frequencies
float32[] rhos
int32 candidate          # -1 = none
float32 dwell            # 0..1
float32 engagement_z
string gate              # off | soft | required
bool signal_ok
string quality           # GOOD | BAD SIGNAL (...)
string source            # SIMULATED | REPLAYED | REAL  (D18: always shown)
float32 threshold
bool no_control
float32 sample_rate      # measured LSL rate
string table_hash        # frequency table version (must match the headset)
bool profile_loaded
