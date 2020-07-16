/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/


#ifndef __EPB_MMBP_H__
#define __EPB_MMBP_H__

#include <stdint.h>
#include <stdbool.h>
#include "epb.h"

typedef enum
{
	ECI_none = 0,
	ECI_req_auth = 10001,
	ECI_req_sendData = 10002,
	ECI_req_init = 10003,
	ECI_resp_auth = 20001,
	ECI_resp_sendData = 20002,
	ECI_resp_init = 20003,
	ECI_push_recvData = 30001,
	ECI_push_switchView = 30002,
	ECI_push_switchBackgroud = 30003,
	ECI_err_decode = 29999
} EmCmdId;

typedef enum
{
	EEC_system = -1,
	EEC_needAuth = -2,
	EEC_sessionTimeout = -3,
	EEC_decode = -4,
	EEC_deviceIsBlock = -5,
	EEC_serviceUnAvalibleInBackground = -6,
	EEC_deviceProtoVersionNeedUpdate = -7,
	EEC_phoneProtoVersionNeedUpdate = -8,
	EEC_maxReqInQueue = -9,
	EEC_userExitWxAccount = -10
} EmErrorCode;

typedef enum
{
	EAM_md5 = 1,
	EAM_macNoEncrypt = 2
} EmAuthMethod;

typedef enum
{
	EIRFF_userNickName = 0x1,
	EIRFF_platformType = 0x2,
	EIRFF_model = 0x4,
	EIRFF_os = 0x8,
	EIRFF_time = 0x10,
	EIRFF_timeZone = 0x20,
	EIRFF_timeString = 0x40
} EmInitRespFieldFilter;

typedef enum
{
	EIS_deviceChat = 1,
	EIS_autoSync = 2
} EmInitScence;

typedef enum
{
	EPT_ios = 1,
	EPT_andriod = 2,
	EPT_wp = 3,
	EPT_s60v3 = 4,
	EPT_s60v5 = 5,
	EPT_s40 = 6,
	EPT_bb = 7
} EmPlatformType;

typedef enum
{
	EDDT_manufatureSvr = 0,
	EDDT_wxWristBand = 1,
	EDDT_wxDeviceHtmlChatView = 10001
} EmDeviceDataType;

typedef enum
{
	ESVO_enter = 1,
	ESVO_exit = 2
} EmSwitchViewOp;

typedef enum
{
	EVI_deviceChatView = 1,
	EVI_deviceChatHtmlView = 2
} EmViewId;

typedef enum
{
	ESBO_enterBackground = 1,
	ESBO_enterForground = 2,
	ESBO_sleep = 3
} EmSwitchBackgroundOp;

typedef struct
{
	void *none;
} BaseRequest;

typedef struct
{
	int32_t err_code;
	bool has_err_msg;
	EString err_msg;
} BaseResponse;

typedef struct
{
	void *none;
} BasePush;

typedef struct
{
	BaseRequest *base_request;
	bool has_md5_device_type_and_device_id;
	Bytes md5_device_type_and_device_id;
	int32_t proto_version;
	int32_t auth_proto;
	EmAuthMethod auth_method;
	bool has_aes_sign;
	Bytes aes_sign;
	bool has_mac_address;
	Bytes mac_address;
	bool has_time_zone;
	eString time_zone;
	bool has_language;
	eString language;
	bool has_device_name;
	eString device_name;
} AuthRequest;

typedef struct
{
	BaseResponse *base_response;
	CBytes aes_session_key;
} AuthResponse;

typedef struct
{
	BaseRequest *base_request;
	bool has_resp_field_filter;
	Bytes resp_field_filter;
	bool has_challenge;
	Bytes challenge;
} InitRequest;

typedef struct
{
	BaseResponse *base_response;
	uint32_t user_id_high;
	uint32_t user_id_low;
	bool has_challeange_answer;
	uint32_t challeange_answer;
	bool has_init_scence;
	EmInitScence init_scence;
	bool has_auto_sync_max_duration_second;
	uint32_t auto_sync_max_duration_second;
	bool has_user_nick_name;
	EString user_nick_name;
	bool has_platform_type;
	EmPlatformType platform_type;
	bool has_model;
	EString model;
	bool has_os;
	EString os;
	bool has_time;
	int32_t time;
	bool has_time_zone;
	int32_t time_zone;
	bool has_time_string;
	EString time_string;
} InitResponse;

typedef struct
{
	BaseRequest *base_request;
	Bytes data;
	bool has_type;
	EmDeviceDataType type;
} SendDataRequest;

typedef struct
{
	BaseResponse *base_response;
	bool has_data;
	CBytes data;
} SendDataResponse;

typedef struct
{
	BasePush *base_push;
	CBytes data;
	bool has_type;
	EmDeviceDataType type;
} RecvDataPush;

typedef struct
{
	BasePush *base_push;
	EmSwitchViewOp switch_view_op;
	EmViewId view_id;
} SwitchViewPush;

typedef struct
{
	BasePush *base_push;
	EmSwitchBackgroundOp switch_background_op;
} SwitchBackgroudPush;

typedef struct  
{       
    int32_t Step;
    int32_t Timestamp;
    int32_t TimeStampRtcYear;
    int32_t TimeStampRtcMonth;
    int32_t TimeStampRtcDay;
    int32_t TimeStampRtcHour;
    int32_t TimeStampRtcMinute;
    int32_t TimeStampRtcSecond;
}StepDataItem;
 
typedef struct  
{    
     StepDataItem *stepdata_item;
     Bytes ExtData;    
}WristbandRequest;

typedef struct  
{
     BaseResponse *BaseResp;
}WristBandResponse; 

//typedef struct  
//{
 
//}WristBandPush;

BaseResponse *epb_unpack_base_response(const uint8_t *buf, int buf_len);
void epb_unpack_base_response_free(BaseResponse *response);
int epb_auth_request_pack_size(AuthRequest *request);
int epb_pack_auth_request(AuthRequest *request, uint8_t *buf, int buf_len);
AuthResponse *epb_unpack_auth_response(const uint8_t *buf, int buf_len);
void epb_unpack_auth_response_free(AuthResponse *response);
int epb_init_request_pack_size(InitRequest *request);
int epb_pack_init_request(InitRequest *request, uint8_t *buf, int buf_len);
InitResponse *epb_unpack_init_response(const uint8_t *buf, int buf_len);
void epb_unpack_init_response_free(InitResponse *response);
int epb_send_data_request_pack_size(SendDataRequest *request);
int epb_pack_send_data_request(SendDataRequest *request, uint8_t *buf, int buf_len);
SendDataResponse *epb_unpack_send_data_response(const uint8_t *buf, int buf_len);
void epb_unpack_send_data_response_free(SendDataResponse *response);
RecvDataPush *epb_unpack_recv_data_push(const uint8_t *buf, int buf_len);
void epb_unpack_recv_data_push_free(RecvDataPush *push);
SwitchViewPush *epb_unpack_switch_view_push(const uint8_t *buf, int buf_len);
void epb_unpack_switch_view_push_free(SwitchViewPush *push);
SwitchBackgroudPush *epb_unpack_switch_backgroud_push(const uint8_t *buf, int buf_len);
void epb_unpack_switch_backgroud_push_free(SwitchBackgroudPush *push);

#endif
