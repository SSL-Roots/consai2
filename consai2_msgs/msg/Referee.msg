# This msg file is copied from 'referee.proto'

# The UNIX timestamp when the packet was sent, in microseconds.
# Divide by 1,000,000 to get a time_t.
uint64 packet_timestamp

# These are the "coarse" stages of the game.
uint8 stage

# If the stage runs over its specified time, this value
# becomes negative.
int32 stage_time_left

# These are the "fine" states of play on the field.
uint8 command

# The number of commands issued since startup (mod 2^32).
uint32 command_counter

# The UNIX timestamp when the command was issued, in microseconds.
# This value changes only when a new command is issued, not on each packet.
uint64 command_timestamp

# Information about the two teams.
RefereeTeamInfo yellow
RefereeTeamInfo blue

# The coordinates of the Designated Position. These are measured in
# millimetres and correspond to SSL-Vision coordinates. These fields are
# always either both present (in the case of a ball placement command) or
# both absent (in the case of any other command).
geometry_msgs/Point designated_position

# Information about the direction of play.
# True, if the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system
# Obviously, the yellow team will play on the opposide half
# For compatibility, this field is optional
bool blue_team_on_positive_half

# The game event that caused the referee command
RefereeGameEvent game_event
