
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/07/05
* Description        : CH554 Type-Cʹ��
��ģʽ(DFP) ��֧�ּ��Typc-C�豸�Ĳ�Σ�֧���豸������ͷ�����,
              ֧��Ĭ�Ϲ��������1.5A��������Լ�3A�Ĺ����������;
��ģʽ(UFP) ������豸�������������ɼ����������������
							Ĭ�Ϲ��������1.5A��������Լ�3A�������
DFP (Downstream Facing Port) Host��
UFP (Upstream Facing Port)   Dev��							 				 
*******************************************************************************/

#include "..\Public\CH554.H"                                                  
#include "..\Public\Debug.H"
#include "Type_C.H"
#include "stdio.h"
#include <string.h>

#pragma  NOAREGS

void main( ) 
{
    UINT16 j = 0;
    CfgFsys( );                                                                //CH554ʱ��ѡ������   
    mDelaymS(20);                                                              //�޸���Ƶ�����Լ���ʱ�ȴ�оƬ�����ȶ�
    mInitSTDIO( );                                                             //����0��ʼ��
    printf("start ...\n"); 
#ifdef   TYPE_C_DFP                                                            //Type-C������������壬֪ͨ�豸������������
    TypeC_DFP_Init(1);                                                         //������������Ĭ�ϵ���
		while(1){
			j = TypeC_DFP_Insert();
			if( j == UCC_DISCONNECT ){
				printf("UFP disconnect...\n");   
			}
			else if( j == UCC1_CONNECT ){
				printf("UFP forward insertion...\n");                                  //�������  
			}
			else if( j == UCC2_CONNECT ){
				printf("UFP reverse insertion...\n");                                  //�������
			}
			else if( j == UCC_CONNECT ){
				printf("UFP connect...\n");   
			}
      mDelaymS(500);                                                           //��ʱ�����壬ģ�ⵥƬ��ִ����������
		}	
#endif
		
#ifdef   TYPE_C_UFP
		TypeC_UPF_PDInit();                                                        //UPF��ʼ��
while(1){		
   j = TypeC_UPF_PDCheck();                                                    //��������Ĺ�������
   if(j == UPF_PD_Normal){
      printf("DFP defaultPower...\n");   
	 }	
   if(j == UPF_PD_1A5){
      printf("DFP 1.5A...\n");   
	 }	  
   if(j == UPF_PD_3A){
      printf("DFP 3.0...\n");   
	 }	
   if(j == UPD_PD_DISCONNECT){
      printf("disconnect...\n");   
	 }
		mDelaymS( 500 );                                                           //��ʱ�����壬ģ�ⵥƬ��ִ����������
 }
#endif
   while(1);
}