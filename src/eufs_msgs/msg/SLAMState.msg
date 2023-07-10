bool loop_closed          # True after loop closure
int16 laps
string status             # Dump for any logging purposes. Eg: "particle covariance high; can't localise"

# Possible states of the algorithm
int8 NO_INPUTS=0          # No inputs have been received yet
int8 MAPPING=1            # Building map
int8 LOCALISING=2         # Loop closed, now localising only
int8 TIMEOUT=3            # Input topics timedout
int8 RECOMMENDS_FAILURE=4 # If true, critical failure!  Recommends emergency shutdown

int8 state                # States enumerated above
