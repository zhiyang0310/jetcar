#include <stdio.h>
#include "../include/CSerialComp.h"   
   
CSerialComp::CSerialComp()   
{   
   tty_name = (char*)"/dev/ttyACM0";   
   construct();   
   serialOpen();   
}   
   
void CSerialComp::construct(void)   
{   
    Serial_Fd = open(tty_name,O_RDWR| O_NOCTTY | O_NDELAY);   
    if(Serial_Fd == -1)   
    {   
        perror("serial_fd:");   
    }   
    portOpen = true;   
    setBaudRate(BAUD9600);   
    setDataBits(DATA_8);   
    setStopBits(STOP_1);   
    setParity(PAR_NONE);   
    setFlowControl(FLOW_OFF);   
   // setTimeout(0, 500);   
   
}   
   
void  CSerialComp::setBaudRate(BaudRateType baudRate)   
{   
    if (portOpen)   
    {   
        switch (baudRate)   
        {   
   
            case BAUD300:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B300;   
#else   
                cfsetispeed(&Posix_CommConfig, B300);   
                cfsetospeed(&Posix_CommConfig, B300);   
#endif   
                break;   
   
   
            case BAUD600:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B600;   
#else   
                cfsetispeed(&Posix_CommConfig, B600);   
                cfsetospeed(&Posix_CommConfig, B600);   
#endif   
                break;   
   
   
            case BAUD1200:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B1200;   
#else   
                cfsetispeed(&Posix_CommConfig, B1200);   
                cfsetospeed(&Posix_CommConfig, B1200);   
#endif   
                break;   
   
   
            case BAUD2400:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B2400;   
#else   
                cfsetispeed(&Posix_CommConfig, B2400);   
                cfsetospeed(&Posix_CommConfig, B2400);   
#endif   
                break;   
   
   
            case BAUD4800:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B4800;   
#else   
                cfsetispeed(&Posix_CommConfig, B4800);   
                cfsetospeed(&Posix_CommConfig, B4800);   
#endif   
                break;   
   
   
            case BAUD9600:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B9600;   
#else   
                cfsetispeed(&Posix_CommConfig, B9600);   
                cfsetospeed(&Posix_CommConfig, B9600);   
#endif   
                break;   
   
   
            case BAUD19200:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B19200;   
#else   
                cfsetispeed(&Posix_CommConfig, B19200);   
                cfsetospeed(&Posix_CommConfig, B19200);   
#endif   
                break;   
   
   
            case BAUD38400:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B38400;   
#else   
                cfsetispeed(&Posix_CommConfig, B38400);   
                cfsetospeed(&Posix_CommConfig, B38400);   
#endif   
                break;   
   
   
            case BAUD57600:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B57600;   
#else   
                cfsetispeed(&Posix_CommConfig, B57600);   
                cfsetospeed(&Posix_CommConfig, B57600);   
#endif   
                break;   
   
            case BAUD115200:   
#ifdef CBAUD   
                Posix_CommConfig.c_cflag&=(~CBAUD);   
                Posix_CommConfig.c_cflag|=B115200;   
#else   
                cfsetispeed(&Posix_CommConfig, B115200);   
                cfsetospeed(&Posix_CommConfig, B115200);   
#endif   
                break;   
   
        tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
      }   
    }   
   
}   
   
void CSerialComp::setFlowControl(FlowType flow)   
{   
    if (Settings.FlowControl!=flow)   
    {   
        Settings.FlowControl=flow;   
    }   
    if (portOpen)   
    {   
        switch(flow)   
        {   
   
   
            case FLOW_OFF:   
                Posix_CommConfig.c_cflag&=(~CRTSCTS);   
                Posix_CommConfig.c_iflag&=(~(IXON|IXOFF|IXANY));   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
            case FLOW_XONXOFF:   
                Posix_CommConfig.c_cflag&=(~CRTSCTS);   
                Posix_CommConfig.c_iflag|=(IXON|IXOFF|IXANY);   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
            case FLOW_HARDWARE:   
                Posix_CommConfig.c_cflag|=CRTSCTS;   
                Posix_CommConfig.c_iflag&=(~(IXON|IXOFF|IXANY));   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
        }   
    }   
   
}   
   
   
void CSerialComp::setDataBits(DataBitsType dataBits)   
{   
     if (Settings.DataBits!=dataBits)   
     {   
        Settings.DataBits=dataBits;   
    }   
    if (portOpen)   
    {   
        switch(dataBits)   
        {   
   
            case DATA_5:   
                Settings.DataBits=dataBits;   
                Posix_CommConfig.c_cflag&=(~CSIZE);   
                Posix_CommConfig.c_cflag|=CS5;   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
   
            case DATA_6:   
                 Settings.DataBits=dataBits;   
                 Posix_CommConfig.c_cflag&=(~CSIZE);   
                 Posix_CommConfig.c_cflag|=CS6;   
                 tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
            case DATA_7:   
                Settings.DataBits=dataBits;   
                Posix_CommConfig.c_cflag&=(~CSIZE);   
                Posix_CommConfig.c_cflag|=CS7;   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
            case DATA_8:   
                Settings.DataBits=dataBits;   
                Posix_CommConfig.c_cflag&=(~CSIZE);   
                Posix_CommConfig.c_cflag|=CS8;   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
        }   
    }   
   
}   
   
void CSerialComp::setStopBits(StopBitsType stopBits)   
{   
    if (Settings.StopBits!=stopBits)   
    {   
        Settings.StopBits=stopBits;   
    }   
    if (portOpen)   
    {   
        switch (stopBits) {   
   
   
            case STOP_1:   
                Settings.StopBits=stopBits;   
                Posix_CommConfig.c_cflag&=(~CSTOPB);   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
            case STOP_2:   
                if (Settings.DataBits==DATA_5) {   
                    printf("Posix_QextSerialPort: 2 stop bits cannot be used with 5 data bits");   
                }   
                else {   
                    Settings.StopBits=stopBits;   
                    Posix_CommConfig.c_cflag|=CSTOPB;   
                    tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                }   
                break;   
        }   
    }   
   
}   
   
void CSerialComp::setParity(ParityType parity)   
{   
    if (Settings.Parity!=parity)   
    {   
        Settings.Parity=parity;   
    }   
    if (portOpen)   
    {   
        switch (parity) {   
   
   
            case PAR_SPACE:   
                if (Settings.DataBits==DATA_8) {   
                    printf("CSerialComp:  Space parity is only supported in POSIX with 7 or fewer data bits");   
                }   
                else {   
   
   
                    Posix_CommConfig.c_cflag&=~(PARENB|CSIZE);   
                    switch(Settings.DataBits) {   
                        case DATA_5:   
                            Settings.DataBits=DATA_6;   
                            Posix_CommConfig.c_cflag|=CS6;   
                            break;   
   
                        case DATA_6:   
                            Settings.DataBits=DATA_7;   
                            Posix_CommConfig.c_cflag|=CS7;   
                            break;   
   
                        case DATA_7:   
                            Settings.DataBits=DATA_8;   
                            Posix_CommConfig.c_cflag|=CS8;   
                            break;   
   
                        case DATA_8:   
                            break;   
                    }   
                    tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                }   
                break;   
   
            case PAR_NONE:   
                Posix_CommConfig.c_cflag&=(~PARENB);   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
   
            case PAR_EVEN:   
                Posix_CommConfig.c_cflag&=(~PARODD);   
                Posix_CommConfig.c_cflag|=PARENB;   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
   
   
            case PAR_ODD:   
                Posix_CommConfig.c_cflag|=(PARENB|PARODD);   
                tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
                break;   
        }   
    }   
   
}   
   
bool  CSerialComp::serialOpen()   
{   
        tcgetattr(Serial_Fd, &Posix_CommConfig);   
        Posix_CommConfig.c_cflag|=CREAD|CLOCAL;   
        Posix_CommConfig.c_lflag&=(~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG));   
        Posix_CommConfig.c_iflag&=(~(INPCK|IGNPAR|PARMRK|ISTRIP|ICRNL|IXANY));   
        Posix_CommConfig.c_oflag&=(~OPOST);   
        Posix_CommConfig.c_cc[VMIN]=0;   
        Posix_CommConfig.c_cc[VINTR] = _POSIX_VDISABLE;   
        Posix_CommConfig.c_cc[VQUIT] = _POSIX_VDISABLE;   
        Posix_CommConfig.c_cc[VSTART] = _POSIX_VDISABLE;   
        Posix_CommConfig.c_cc[VSTOP] = _POSIX_VDISABLE;   
        Posix_CommConfig.c_cc[VSUSP] = _POSIX_VDISABLE;   
       // setBaudRate(Settings.BaudRate);   
       // setDataBits(Settings.DataBits);   
        //setStopBits(Settings.StopBits);   
        //setParity(Settings.Parity);   
       // setFlowControl(Settings.FlowControl);   
        //setTimeout(Posix_Copy_Timeout.tv_sec, Posix_Copy_Timeout.tv_usec);   
        tcsetattr(Serial_Fd, TCSAFLUSH, &Posix_CommConfig);   
   
     return  portOpen;   
   
}   

fd_set  fd_array;   
int recvCount = 0;
int  CSerialComp::serialRead(char *chr)   
{  
    FD_ZERO(&fd_array);   
    FD_SET(Serial_Fd,&fd_array);   
    select(Serial_Fd+1,&fd_array,NULL,NULL,NULL);   
    recvCount = read(Serial_Fd,chr,1); 
    return recvCount; 
}   
   
int CSerialComp::serialWrite(char *buffer,int size)   
{   
    int writeCount = 0;   
    if(size <= 0)   
        return 0;   
    writeCount = write(Serial_Fd,buffer,size);   
    return writeCount;   
}   
   
CSerialComp::~CSerialComp()   
{   
    close(Serial_Fd);   
}