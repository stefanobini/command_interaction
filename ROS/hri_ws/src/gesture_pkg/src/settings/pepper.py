from dotmap import DotMap
pepper = DotMap()

############## PEPPER SETUP ###############
#pepper.ip = "10.0.1.200"
pepper.ip = "10.0.1.230"
pepper.port = 9559

############ PEPPER MICROPHONE ############
pepper.mic.sr = 16000
pepper.mic.mic_id = 3
pepper.mic.deinterleaved = 0

############## PEPPER SPEECH ##############
pepper.speech.language = "English"
pepper.speech.animated = False
pepper.speech.rspd = 88
pepper.speech.comma_pause = 150
pepper.speech.period_pause = 300

############## PEPPER MOTION ##############
pepper.motion.head_period = 0.3

############## PEPPER EVENTS ##############
pepper.events = [
    "ALBasicAwareness/HumanLost:ALMemory",
    "ALBasicAwareness/HumanTracked:ALMemory",
    "BackBumperPressed:ALMemory",    
    "HandLeftBackTouched:ALMemory",
    "HandRightBackTouched:ALMemory",    
    "LeftBumperPressed:ALMemory",    
    "ALTextToSpeech/TextDone:ALMemory"   
]