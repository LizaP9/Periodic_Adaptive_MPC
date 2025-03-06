uint8[2] head
uint8 frameReserve
uint32[2] SN
uint32[2] version
uint16 bandWidth
uint8 gaitType
uint8 speedLevel
float32[2] position
float32[3] euler	   
float32[2] velocity
float32 yawSpeed
BmsCmd bms

uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have

#uint8 bandWidth                 # Old version Aliengo does not have
uint8 mode
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed	
float32 bodyHeight
float32 footRaiseHeight
float32 yaw
float32 pitch
float32 roll
LED[4] led
uint8[40] wirelessRemote
uint8[40] AppRemote             # Old version Aliengo does not have
uint32 reserve                  # Old version Aliengo does not have
int32 crc