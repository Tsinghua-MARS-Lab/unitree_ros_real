
uint8 levelFlag
uint16 commVersion

uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode
float32 progress

IMU imu
uint8 gaitType		   
float32 footRaiseHeight		  
float32[3] position 
float32 bodyHeight			  
float32[3] velocity 
float32 yawSpeed				   
Cartesian[4] footPosition2Body 
Cartesian[4] footSpeed2Body	
int16[4] footForce
int16[4] footForceEst
WirelessRemote wirelessRemote
uint32 reserve

uint32 crc