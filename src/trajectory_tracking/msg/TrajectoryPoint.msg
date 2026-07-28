# A single time-parameterized point on the planned trajectory.

float64 x                  # metres, in the 'odom' frame
float64 y                  # metres, in the 'odom' frame
float64 theta               # heading (yaw), radians, normalized to [-pi, pi]
float64 velocity            # commanded linear speed at this point, m/s
float64 time_from_start      # seconds since trajectory start (t0 = 0)
