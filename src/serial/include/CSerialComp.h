#ifndef _CSERIALCOMP_H_ 
#define _CSERIALCOMP_H_ 
 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <stdlib.h> 
#include <sys/select.h> 
#include <string.h> 
 
typedef enum _FlowType { 
    FLOW_OFF, 
    FLOW_HARDWARE, 
    FLOW_XONXOFF 
} FlowType; 
 
typedef enum _ParityType { 
    PAR_NONE, 
    PAR_ODD, 
    PAR_EVEN, 
    PAR_SPACE 
} ParityType; 
 
typedef enum _DataBitsType { 
    DATA_5, 
    DATA_6, 
    DATA_7, 
    DATA_8 
} DataBitsType; 
 
typedef enum _StopBitsType { 
    STOP_1, 
    STOP_2 
} StopBitsType; 
 
typedef enum _BaudRateType { 
    BAUD300, 
    BAUD600, 
    BAUD1200, 
    BAUD2400, 
    BAUD4800, 
    BAUD9600, 
    BAUD19200, 
    BAUD38400, 
    BAUD57600, 
    BAUD115200, 
} BaudRateType; 
 
/*structure to contain port settings*/ 
typedef struct _PortSettings { 
    FlowType FlowControl; 
    ParityType Parity; 
    DataBitsType DataBits; 
    StopBitsType StopBits; 
    BaudRateType BaudRate; 
    unsigned long Timeout_Sec; 
    unsigned long Timeout_Millisec; 
} PortSettings; 
 
 
class  CSerialComp 
{ 
	public: 
		CSerialComp(); 
		~CSerialComp(); 
         bool serialOpen(); 
		 void setFlowControl(FlowType); 
		 void setParity(ParityType); 
		 void setDataBits(DataBitsType); 
		 void setStopBits(StopBitsType); 
		 void setBaudRate(BaudRateType); 
		 int serialRead(char *chr); 
		 int  serialWrite(char *buffer,int size); 
	protected: 
		int  Serial_Fd; 
        bool portOpen; 
        char *tty_name; 
        char recv_buffer[50]; 
        PortSettings  Settings; 
		struct termios Posix_CommConfig; 
		struct timeval Posix_Timeout; 
		struct timeval Posix_Copy_Timeout; 
		void construct(void); 
};

#endif