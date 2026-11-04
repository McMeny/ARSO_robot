# One obstacle the walker is tracking, and where it is in the hazard loop.

uint8 WAITING = 0      # detected, not (yet) relevant, or queued for a free frequency
uint8 PROBING = 1      # tagged and flickering, waiting for the EEG
uint8 REGISTERED = 2   # the EEG showed the user saw it - handled gently, tag fades
uint8 MISSED = 3       # not registered by the decision distance - escalated
uint8 PASSED = 4       # behind the walker

uint32 id
string name                    # e.g. the model name in the simulated store
string kind                    # 'box' or 'cylinder'
geometry_msgs/Point position   # centre, map frame
float32 size_x                 # footprint of the object (m), in its own frame
float32 size_y
float32 height
float32 yaw                    # orientation of the object in the map frame (rad)
float32 path_distance          # distance ahead along the walker's path (m), negative = behind
float32 lateral                # gap between the path and the object's edge (m), 0 = on the path
bool relevant                  # on or near the path and within the probe distance
uint8 state
float32 frequency              # flicker frequency (Hz), 0 = no tag
float32 contrast               # flicker contrast (0..1); higher = the stronger cue
float32 confidence             # latest EEG score for this hazard (decoder units, 1 = noise level)
