

#define UART1_INTERRUPT   0          //�����Ƿ�ʹ�ý����жϽ�������
#define UART1_PINMAP      0          //���崮��1�Ƿ�������ӳ�䡣������ѡ��P16(R)��P17(T)������ѡ��P34(R)��P32(T)


extern void   UART1Init( );
extern UINT8  CH554UART1RcvByte( );
extern void   CH554UART1SendByte(UINT8 SendDat);