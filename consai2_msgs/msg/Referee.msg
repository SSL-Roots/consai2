# This msg file is copied from 'referee.proto'

# These are the "coarse" stages of the game.
uint8 stage

# These are the "fine" states of play on the field.
uint8 command

# The number of commands issued since startup (mod 2^32).
uint32 command_counter

# Information about the two teams.
RefereeTeamInfo yellow
RefereeTeamInfo blue

# The coordinates of the Designated Position. These are measured in
# millimetres and correspond to SSL-Vision coordinates. These fields are
# always either both present (in the case of a ball placement command) or
# both absent (in the case of any other command).
geometry_msgs/Point designated_position

# The game event that caused the referee command
RefereeGameEvent game_event
