# This msg file is copied from 'game_event.proto'

# the game event type that happened
uint8 game_event_type

# information about an originator
uint8 originator_team
uint32 originator_bot_id

# a message describing further details of this game event
string message
