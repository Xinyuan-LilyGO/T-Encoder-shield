#include "CH554.H"
#include "Debug.H"
#include "stdio.h"
#include "GPIO.H"
#include <string.h>
#include "SPI.H"
//#include "UART1.H"

#pragma  NOAREGS

//#define PIN_USB_DP   P3.6
//#define PIN_USB_DM   P3.7
//#define PIN_UART_TX  P3.1
//#define PIN_UART_RX  P3.0
//#define PIN_APA_CLK  P1.7
//#define PIN_APA_SDA  P1.5
//#define PIN_BTN_1    P1.4
//#define PIN_BTN_2    P3.2


//#define Fullspeed
#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define ENDP1_IN_SIZE 			8
#define ENDP2_IN_SIZE 			4

UINT8X  Ep0Buffer[MIN(64, THIS_ENDP0_SIZE + 2)]  _at_ 0x0000; //�˵�0 OUT&IN��������������ż��ַ
UINT8X  Ep1Buffer[MIN(64, ENDP1_IN_SIZE + 2)] _at_ MIN(64, THIS_ENDP0_SIZE + 2); //�˵�1 IN������,������ż��ַ
UINT8X  Ep2Buffer[MIN(64, ENDP2_IN_SIZE + 2)] _at_ (MIN(64, THIS_ENDP0_SIZE + 2) + MIN(64, ENDP1_IN_SIZE + 2)); //�˵�2 IN������,������ż��ַ
UINT8   SetupReq, SetupLen, Ready, Count, FLAG, UsbConfig;
PUINT8  pDescr;                                                                //USB���ñ�־
USB_SETUP_REQ   SetupReqBuf;                                                   //�ݴ�Setup��
sbit Key1 = P1 ^ 4;
sbit Key2 = P3 ^ 2;
//7 led
#define APA102_BRIGHTNESS 5
#define APA102_QUANTITY 7
#define APA102_COL 5
#define APA102_COL_QUANTITY 2//Quantity per column

char apa102_buf[] = {0x00, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XE1, 0x00, 0x00, 0x00, \
                     0XFF, 0xFF, 0XFF, 0XFF\
                    };

char row_table[] = {2, 2, 1, 1, 1 };//How many lights are in each column from left to right
char col_table[APA102_COL][APA102_COL_QUANTITY] =
{
    3, 7, \
    4, 6, \
    5, 0, \
    2, 0, \
    1, 0, \
};


#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma  NOAREGS
/*�豸������*/
UINT8C DevDesc[18] = {0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, THIS_ENDP0_SIZE,
                      0x3d, 0x41, 0x07, 0x21, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x01
                     };
UINT8C CfgDesc[59] =
{
    0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32,     //����������
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,     //�ӿ�������,����
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3e, 0x00,     //HID��������
    0x07, 0x05, 0x81, 0x03, ENDP1_IN_SIZE, 0x00, 0x0a,           //�˵�������
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,     //�ӿ�������,���
    0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00,     //HID��������
    0x07, 0x05, 0x82, 0x03, ENDP2_IN_SIZE, 0x00, 0x0a            //�˵�������
};
/*�ַ���������*/
/*HID�౨��������*/
UINT8C KeyRepDesc[62] =
{
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x03, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x03, 0x91, 0x02,
    0x95, 0x05, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x26, 0xff, 0x00, 0x05, 0x07, 0x19,
    0x00, 0x29, 0x91, 0x81, 0x00, 0xC0
};
UINT8C MouseRepDesc[52] =
{
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01,
    0xA1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03,
    0x81, 0x02, 0x75, 0x05, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
    0x81, 0x06, 0xC0, 0xC0
};
/*�������*/
UINT8 HIDMouse[4] = {0x0, 0x0, 0x0, 0x0};
/*��������*/
UINT8 HIDKey[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};




/*******************************************************************************
* Function Name  : CH554USBDevWakeup()
* Description    : CH554�豸ģʽ��������������K�ź�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554USBDevWakeup( )
{
    #ifdef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED;
    mDelaymS(2);
    UDEV_CTRL &= ~bUD_LOW_SPEED;
    #else
    UDEV_CTRL &= ~bUD_LOW_SPEED;
    mDelaymS(2);
    UDEV_CTRL |= bUD_LOW_SPEED;
    #endif
}

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB�豸ģʽ����,�豸ģʽ�������շ��˵����ã��жϿ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;                                                           // ���趨USB�豸ģʽ
    UDEV_CTRL = bUD_PD_DIS;                                                    // ��ֹDP/DM��������
    #ifndef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED;                                                //ѡ�����1.5Mģʽ
    USB_CTRL |= bUC_LOW_SPEED;
    #else
    UDEV_CTRL &= ~bUD_LOW_SPEED;                                               //ѡ��ȫ��12Mģʽ��Ĭ�Ϸ�ʽ
    USB_CTRL &= ~bUC_LOW_SPEED;
    #endif

    UEP2_DMA = Ep2Buffer;                                                      //�˵�2���ݴ����ַ
    UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //�˵�2����ʹ�� 64�ֽڻ�����

    UEP0_DMA = Ep0Buffer;                                                      //�˵�0���ݴ����ַ
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //�˵�0��64�ֽ��շ�������

    UEP1_DMA = Ep1Buffer;                                                      //�˵�1���ݴ����ַ
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //�˵�1����ʹ�� 64�ֽڻ�����

    USB_DEV_AD = 0x00;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
    UDEV_CTRL |= bUD_PORT_EN;                                                  // ����USB�˿�
    USB_INT_FG = 0xFF;                                                         // ���жϱ�־
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB�豸ģʽ�˵�1���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey));                              //�����ϴ�����
    UEP1_T_LEN = sizeof(HIDKey);                                             //�ϴ����ݳ���
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //������ʱ�ϴ����ݲ�Ӧ��ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB�豸ģʽ�˵�2���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDMouse, sizeof(HIDMouse));                              //�����ϴ�����
    UEP2_T_LEN = sizeof(HIDMouse);                                              //�ϴ����ݳ���
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                  //������ʱ�ϴ����ݲ�Ӧ��ACK
}
/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB�жϴ�����
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      //USB�жϷ������,ʹ�üĴ�����1
{
    UINT8 len = 0;

    if(UIF_TRANSFER)                                                            //USB������ɱ�־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
            case UIS_TOKEN_IN | 2:                                                  //endpoint 2# �ж϶˵��ϴ�
                UEP2_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
                UEP2_CTRL ^= bUEP_T_TOG;                                            //�ֶ���תͬ����־λ
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
                break;

            case UIS_TOKEN_IN | 1:                                                  //endpoint 1# �ж϶˵��ϴ�
                UEP1_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
                UEP1_CTRL ^= bUEP_T_TOG;                                            //�ֶ���תͬ����־λ
                UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
                FLAG = 1;                                                           /*������ɱ�־*/
                break;

            case UIS_TOKEN_SETUP | 0:                                                //SETUP����
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
                len = USB_RX_LEN;

                if(len == (sizeof(USB_SETUP_REQ)))
                {
                    SetupLen = UsbSetupBuf->wLengthL;

                    if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                    {
                        SetupLen = 0x7F;    // �����ܳ���
                    }

                    len = 0;                                                        // Ĭ��Ϊ�ɹ������ϴ�0����
                    SetupReq = UsbSetupBuf->bRequest;

                    if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID������ */
                    {
                        switch( SetupReq )
                        {
                            case 0x01://GetReport
                                break;

                            case 0x02://GetIdle
                                break;

                            case 0x03://GetProtocol
                                break;

                            case 0x09://SetReport
                                break;

                            case 0x0A://SetIdle
                                break;

                            case 0x0B://SetProtocol
                                break;

                            default:
                                len = 0xFF;  								   /*���֧��*/
                                break;
                        }
                    }
                    else
                    {
                        //��׼����
                        switch(SetupReq)                                        //������
                        {
                            case USB_GET_DESCRIPTOR:
                                switch(UsbSetupBuf->wValueH)
                                {
                                    case 1:                                             //�豸������
                                        pDescr = DevDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                                        len = sizeof(DevDesc);
                                        break;

                                    case 2:                                             //����������
                                        pDescr = CfgDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                                        len = sizeof(CfgDesc);
                                        break;

                                    case 0x22:                                          //����������
                                        if(UsbSetupBuf->wIndexL == 0)                   //�ӿ�0����������
                                        {
                                            pDescr = KeyRepDesc;                        //����׼���ϴ�
                                            len = sizeof(KeyRepDesc);
                                        }
                                        else if(UsbSetupBuf->wIndexL == 1)              //�ӿ�1����������
                                        {
                                            pDescr = MouseRepDesc;                      //����׼���ϴ�
                                            len = sizeof(MouseRepDesc);
                                        }
                                        else
                                        {
                                            len = 0xff;                                 //������ֻ��2���ӿڣ���仰����������ִ��
                                        }

                                        break;

                                    default:
                                        len = 0xff;                                     //��֧�ֵ�������߳���
                                        break;
                                }

                                if ( SetupLen > len )
                                {
                                    SetupLen = len;    //�����ܳ���
                                }

                                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                                memcpy(Ep0Buffer, pDescr, len);                      //�����ϴ�����
                                SetupLen -= len;
                                pDescr += len;
                                break;

                            case USB_SET_ADDRESS:
                                SetupLen = UsbSetupBuf->wValueL;                     //�ݴ�USB�豸��ַ
                                break;

                            case USB_GET_CONFIGURATION:
                                Ep0Buffer[0] = UsbConfig;

                                if ( SetupLen >= 1 )
                                {
                                    len = 1;
                                }

                                break;

                            case USB_SET_CONFIGURATION:
                                UsbConfig = UsbSetupBuf->wValueL;

                                if(UsbConfig)
                                {
                                    #ifdef DE_PRINTF
                                    printf("SET CONFIG.\n");
                                    #endif
                                    Ready = 1;                                      //set config����һ�����usbö����ɵı�־
                                }

                                break;

                            case 0x0A:
                                break;

                            case USB_CLEAR_FEATURE:                                            //Clear Feature
                                if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// �˵�
                                {
                                    switch( UsbSetupBuf->wIndexL )
                                    {
                                        case 0x82:
                                            UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;

                                        case 0x81:
                                            UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;

                                        case 0x01:
                                            UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                            break;

                                        default:
                                            len = 0xFF;                                            // ��֧�ֵĶ˵�
                                            break;
                                    }
                                }

                                if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )// �豸
                                {
                                    break;
                                }
                                else
                                {
                                    len = 0xFF;                                                // ���Ƕ˵㲻֧��
                                }

                                break;

                            case USB_SET_FEATURE:                                              /* Set Feature */
                                if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* �����豸 */
                                {
                                    if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                                    {
                                        if( CfgDesc[ 7 ] & 0x20 )
                                        {
                                            /* ���û���ʹ�ܱ�־ */
                                        }
                                        else
                                        {
                                            len = 0xFF;                                        /* ����ʧ�� */
                                        }
                                    }
                                    else
                                    {
                                        len = 0xFF;                                            /* ����ʧ�� */
                                    }
                                }
                                else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* ���ö˵� */
                                {
                                    if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                                    {
                                        switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                        {
                                            case 0x82:
                                                UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�2 IN STALL */
                                                break;

                                            case 0x02:
                                                UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* ���ö˵�2 OUT Stall */
                                                break;

                                            case 0x81:
                                                UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�1 IN STALL */
                                                break;

                                            default:
                                                len = 0xFF;                               //����ʧ��
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        len = 0xFF;                                   //����ʧ��
                                    }
                                }
                                else
                                {
                                    len = 0xFF;                                      //����ʧ��
                                }

                                break;

                            case USB_GET_STATUS:
                                Ep0Buffer[0] = 0x00;
                                Ep0Buffer[1] = 0x00;

                                if ( SetupLen >= 2 )
                                {
                                    len = 2;
                                }
                                else
                                {
                                    len = SetupLen;
                                }

                                break;

                            default:
                                len = 0xff;                                           //����ʧ��
                                break;
                        }
                    }
                }
                else
                {
                    len = 0xff;                                                   //�����ȴ���
                }

                if(len == 0xff)
                {
                    SetupReq = 0xFF;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
                }
                else if(len)                                                //�ϴ����ݻ���״̬�׶η���0���Ȱ�
                {
                    UEP0_T_LEN = len;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1������Ӧ��ACK
                }
                else
                {
                    UEP0_T_LEN = 0;  //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1,����Ӧ��ACK
                }

                break;

            case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
                switch(SetupReq)
                {
                    case USB_GET_DESCRIPTOR:
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;    //���δ��䳤��
                        memcpy( Ep0Buffer, pDescr, len );                            //�����ϴ�����
                        SetupLen -= len;
                        pDescr += len;
                        UEP0_T_LEN = len;
                        UEP0_CTRL ^= bUEP_T_TOG;                                     //ͬ����־λ��ת
                        break;

                    case USB_SET_ADDRESS:
                        USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                        break;

                    default:
                        UEP0_T_LEN = 0;                                              //״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
                        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                        break;
                }

                break;

            case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
                len = USB_RX_LEN;

                if(SetupReq == 0x09)
                {
                    if(Ep0Buffer[0])
                    {
                        printf("Light on Num Lock LED!\n");
                    }
                    else if(Ep0Buffer[0] == 0)
                    {
                        printf("Light off Num Lock LED!\n");
                    }
                }

                UEP0_CTRL ^= bUEP_R_TOG;                                     //ͬ����־λ��ת
                break;

            default:
                break;
        }

        UIF_TRANSFER = 0;                                                 //д0����ж�
    }

    if(UIF_BUS_RST)                                                       //�豸ģʽUSB���߸�λ�ж�
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = UEP_T_RES_NAK;
        UEP2_CTRL = UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //���жϱ�־
    }

    if (UIF_SUSPEND)                                                     //USB���߹���/�������
    {
        UIF_SUSPEND = 0;

        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //����
        {
            #if DEBUG
            printf( "zz" );                                              //˯��״̬
            #endif
//             while ( XBUS_AUX & bUART0_TX );                              //�ȴ��������
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                      //USB����RXD0���ź�ʱ�ɱ�����
//             PCON |= PD;                                                  //˯��
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = 0x00;
        }
    }
    else                                                                 //������ж�,�����ܷ��������
    {
        USB_INT_FG = 0xFF;                                               //���жϱ�־
//      printf("UnknownInt  N");
    }
}
void HIDValueHandle()
{
    UINT8 i;
    i = _getkey( );
    printf( "%c", (UINT8)i );

    switch(i)
    {
//��������ϴ�ʾ��
        case 'L':                                                        //���
            HIDMouse[0] = 0x01;
            Enp2IntIn();
            HIDMouse[0] = 0;
            break;

        case 'R':                                                        //�Ҽ�
            HIDMouse[0] = 0x02;
            Enp2IntIn();
            HIDMouse[0] = 0;
            break;

//���������ϴ�ʾ��
        case 'A':                                                         //A��
            FLAG = 0;
            HIDKey[2] = 0x1d;                                             //������ʼ
            Enp1IntIn();
            HIDKey[2] = 0;                                                //��������

            while(FLAG == 0)
            {
                ;    /*�ȴ���һ���������*/
            }

            Enp1IntIn();
            break;

        case 'P':                                                         //P��
            FLAG = 0;
            HIDKey[2] = 0x38;
            Enp1IntIn();
            HIDKey[2] = 0;                                                //��������

            while(FLAG == 0)
            {
                ;    /*�ȴ���һ���������*/
            }

            Enp1IntIn();
            break;

        case 'Q':                                                         //Num Lock��
            FLAG = 0;
            HIDKey[2] = 0x39;
            Enp1IntIn();
            HIDKey[2] = 0;                                                //��������

            while(FLAG == 0)
            {
                ;    /*�ȴ���һ���������*/
            }

            Enp1IntIn();
            break;

        default:                                                          //����
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;     //Ĭ��Ӧ��NAK
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;     //Ĭ��Ӧ��NAK
            break;
    }
}


void SendSingleKey(UINT8 val)
{
    FLAG = 0;
    HIDKey[2] = val;                                             //������ʼ
    Enp1IntIn();
    memset(HIDKey, 0, 8);

    while(FLAG == 0)
    {
        ;    /*�ȴ���һ���������*/
    }

    Enp1IntIn();
}

/* https://www.cnblogs.com/guyandianzi/p/9882644.html */
void SendCombinKey(UINT8 com, UINT8 val)
{
    FLAG = 0;
    HIDKey[0] = com;
    HIDKey[2] = val;                                             //������ʼ
    Enp1IntIn();
    memset(HIDKey, 0, 8);
    HIDKey[2] = 0;                                                //��������

    while(FLAG == 0)
    {
        ;    /*�ȴ���һ���������*/
    }

    Enp1IntIn();
}

void SendMouseKey(UINT8 val)
{
    HIDMouse[3] = val;                                             //������
    Enp2IntIn();
    memset(HIDMouse, 0, 4);
}




void change_led(UINT8X num, UINT8X brightness, UINT32X rgb)
{
    UINT8X r, g, b = 0;
    UINT8X head = (num * 4) + 4;
    r = (rgb & 0xFF0000) >> 16;
    g = (rgb & 0xFF00) >> 8;
    b = rgb & 0xFF;

    apa102_buf[head] = (brightness & 0x1F) | 0xE0;
    apa102_buf[head + 1] = b;
    apa102_buf[head + 2] = g;
    apa102_buf[head + 3] = r;
}

UINT32 rainbow(UINT32 value)
{
    UINT8 red   = 0; // Red is the top 5 bits of a 16 bit colour value
    UINT8 green = 0; // Green is the middle 6 bits, but only top 5 bits used here
    UINT8 blue  = 0; // Blue is the bottom 5 bits
    UINT8 sector = 0;
    UINT8 amplit = 0;
    UINT8 tmp = value % 192;
    UINT32 res = 0;
    sector = tmp >> 5;
    amplit = tmp & 0x1F;

    switch (sector)
    {
        case 0:
            red   = 0x1F;
            green = amplit; // Green ramps up
            blue  = 0;
            break;

        case 1:
            red   = 0x1F - amplit; // Red ramps down
            green = 0x1F;
            blue  = 0;
            break;

        case 2:
            red   = 0;
            green = 0x1F;
            blue  = amplit; // Blue ramps up
            break;

        case 3:
            red   = 0;
            green = 0x1F - amplit; // Green ramps down
            blue  = 0x1F;
            break;

        case 4:
            red   = amplit; // Red ramps up
            green = 0;
            blue  = 0x1F;
            break;

        case 5:
            red   = 0x1F;
            green = 0;
            blue  = 0x1F - amplit; // Blue ramps down
            break;
    }

    res = red << 8;
    res = res << 8;
    res |= green << 8 | blue;
    return res;
}

void main(void)
{
    int i, j, k = 0;
    char key1_up, key2_up, key1_down, key2_down, key1_delay, key2_delay = 0;
    char blink_mode = 0;
    UINT32 color = 0;
		UINT8 dat;
//    char *str_ptr;
    CfgFsys( );                                                                //CH554ʱ��ѡ������
    mDelaymS(20);
    mInitSTDIO( );                                                        //����0��ʼ��
//    UART1Init(); /* Initialize T-Encoder uart port */
    Port1Cfg(3, 4);
    Port1Cfg(3, 2);
    printf("start ...\n");

    USBDeviceInit();                                                      //USB�豸ģʽ��ʼ��
    EA = 1;                                                               //����Ƭ���ж�
    UEP1_T_LEN = 0;                                                       //Ԥʹ�÷��ͳ���һ��Ҫ���
    UEP2_T_LEN = 0;                                                       //Ԥʹ�÷��ͳ���һ��Ҫ���
    FLAG = 0;
    Ready = 0;

    SPI_CK_SET(2);
    SPIMasterModeSet(0);

    while(1)
    {

        /* Receive T-Encoder information */
				 dat = CH554UART0RcvByte( );
        if(dat != 0)
        {
            blink_mode++;
						CH554UART0SendByte(dat);
            if(Ready)
            {
//                for(i = 0; i < uart_count; i++)
//                {
                    switch(dat)
                    {
                        case 'L'://L
                            SendMouseKey(1);  //Simulate mouse wheel +1
                            break;

                        case 'R'://R
                            SendMouseKey(-1);  //Simulate mouse wheel -1
                            break;

                        case 'B':
                            blink_mode = ++blink_mode % 3; //toggle lights
                            break;

                        default:
                            break;
                    }
//                }
            }
//            uart_count = 0;
        }


        /* Button press detection */
        if((Ready) && (Key1 == 0) && key1_up == 1) //����
        {
            mDelaymS(10);

            if(Key1 == 0 && key1_down == 0)//����
            {
                SendCombinKey(0x01, 0x19);  //����CTRL+V
CH554UART0SendByte('B');
                blink_mode++;
                key1_down = 1;
                key1_up == 0;
            }
        }
        else if((Key1 == 0) && key1_down == 1)   //����
        {
            mDelaymS(10);

            if(key1_delay++ > 10)
            {
                key1_delay = 0;
                SendCombinKey(0x01, 0x19);
            }
        }
        else
        {
            key1_up = 1;
            key1_down = 0;
        }

        if((Ready) && (Key2 == 0) && key2_up == 1)//����
        {
            mDelaymS(10);

            if(Key2 == 0 && key2_down == 0)//����
            {
                SendCombinKey(0x01, 0x06); //����CTRL+C
//                SendMouseKey(-1);
                blink_mode++;
                key2_down = 1;
                key2_up == 0;
            }
        }
        else if((Key2 == 0) && key2_down == 1)  //����
        {
            mDelaymS(10);

            if(key2_delay++ > 10)
            {
                key2_delay = 0;
                SendCombinKey(0x01, 0x06);
            }
        }
        else
        {
            key2_up = 1;
            key2_down = 0;
        }

        mDelaymS(20);

        /* Modify APA102_BUF parameter */
        if(++j == 193)
            j = 0;

        if(blink_mode == 0)//Show Combination 1
        {
            for(i = 0; i < APA102_QUANTITY; i++)
            {
                color = rainbow(j + (i * 5));
                change_led(i, APA102_BRIGHTNESS, color);
            }
        }
        else if(blink_mode == 1)//Show Combination 2
        {
            color = rainbow(j);

            for(i = 0; i < APA102_QUANTITY; i++)
            {
                change_led(i, APA102_BRIGHTNESS, color);
            }
        }
        else if(blink_mode == 2)//Show Combination 3
        {
            for(i = 0; i < APA102_COL; i++)
            {
                for(k = 0; k < row_table[i]; k++)
                {
                    color = rainbow(j + (i * 15));
                    change_led(col_table[i][k], APA102_BRIGHTNESS, color );
                }
            }
        }
        else
            blink_mode = 0;

        /* Send APA102_BUF parameter */
        for( i = 0; i < (sizeof(apa102_buf) / sizeof(char)); i++)
        {
            CH554SPIMasterWrite(apa102_buf[i]);
        }


    }
}
