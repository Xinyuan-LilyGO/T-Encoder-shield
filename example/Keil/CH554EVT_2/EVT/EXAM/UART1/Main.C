
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.0
* Date               : 2018/07/25
* Description        : CH544 ����1 �շ�                   
*******************************************************************************/
#include "..\Public\CH554.H"                                                  
#include "..\Public\Debug.H"
#include "UART1.H"
#include "stdio.h"
#include <string.h>

#pragma  NOAREGS

void main( ) 
{
#if ( UART1_INTERRUPT == 0) 
	UINT8 dat;
#endif
    CfgFsys( );                                                                //CH554ʱ��ѡ������   
    mDelaymS(20);
    mInitSTDIO( );                                                             //����0���Զ˿ڳ�ʼ��
    printf("start ...\n"); 

    UART1Init( );                                                              //����1��ʼ��
	while(1)
	{
#if ( UART1_INTERRUPT == 0)                                                    //��ѯ��ʽ���յ�һ���ֽڣ���ת����ȥ
		dat = CH554UART1RcvByte( );                                            //�������ȣ�ֱ���յ�һ���ֽ�
		CH554UART1SendByte(dat);                                               //��ͨ������1����ȥ
#endif		
	}

}