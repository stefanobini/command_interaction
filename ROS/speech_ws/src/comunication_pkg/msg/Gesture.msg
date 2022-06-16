# 

# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)
string id
# Identify the Entity/topic type (Speech)
string type
#Two-integer timestamp that is expressed as:
# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
string timestamp
# Identify the command detected by Speech/Command system, it contains the following information:
# * Numeric identifier associated with the command
# * Textual description of the gesture, in English
# * Textual description of the gesture, in Italian
Command command
# Value between 0 and 1 which indicates the reliability in the command identification and classification
float32 confidence