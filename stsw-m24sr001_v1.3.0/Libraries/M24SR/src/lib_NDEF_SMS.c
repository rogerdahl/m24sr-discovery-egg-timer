/**
  ******************************************************************************
  * @file    lib_NDEF_SMS.c
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    20-November-2013
  * @brief   This file help to manage NDEF file that represent SMS.
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MMY-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF_SMS.h"


/** @addtogroup NFC_libraries
 * 	@{
 *	@brief  <b>This is the library used to manage the content of the TAG (data)
 *          But also the specific feature of the tag, for instance
 *          password, gpo... </b>
 */


/** @addtogroup libNFC_FORUM
  * @{
	*	@brief  This part of the library manage data which follow NFC forum organisation.
  */

/**
 * @brief  This buffer contains the data send/received by TAG
 */
extern uint8_t NDEF_Buffer [NDEF_MAX_SIZE];

/** @defgroup libSMS_Private_Functions
  * @{
  */

static void NDEF_FillSMSStruct( u8* pPayload, u32 PayloadSize, sSMSInfo *pSMSStruct);
static void NDEF_ReadURI_SMS ( sRecordInfo *pRecordStruct, sSMSInfo *pSMSStruct );

/**
  * @brief  This fonction fill SMS structure with information of NDEF message
	* @param	pPayload : pointer on the payload data of the NDEF message
	* @param	PayloadSize : number of data in the payload
	* @param	pSMSStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_FillSMSStruct( u8* pPayload, u32 PayloadSize, sSMSInfo *pSMSStruct)
{
	u8* pLastByteAdd, *pLook4Word, *pEndString ;
	char* pKeyWord;
	u32 SizeOfKeyWord;
	
	pKeyWord = SMS_TYPE_STRING;
	SizeOfKeyWord = SMS_TYPE_STRING_LENGTH;
	
	/* First charactere force to NULL in case not matching found */
	*pSMSStruct->PhoneNumber = 0;
	*pSMSStruct->Message = 0;
	
	/* Interresting information are stored before picture if any */
	/* Moreover picture is not used in this demonstration SW */	
	pLastByteAdd = (u8*)(pPayload + PayloadSize);
		
	pLook4Word = pPayload;
	while( memcmp( pLook4Word, pKeyWord, SizeOfKeyWord) && pLook4Word<pLastByteAdd )
	{
		pLook4Word++;
	}
	
	/* Retrieve phone number */
	if( pLook4Word != pLastByteAdd)
	{	
		pLook4Word += SizeOfKeyWord;
		pEndString = pLook4Word;
		while( memcmp( pEndString, URI_FIRST_DATA_END, URI_FIRST_DATA_END_LENGTH) && pEndString<pLastByteAdd )
		{
			pEndString++;
		}
		if( pEndString != pLastByteAdd)
		{
			memcpy( pSMSStruct->PhoneNumber, pLook4Word, pEndString-pLook4Word);
			/* add end of string charactere */
			pSMSStruct->PhoneNumber[pEndString-pLook4Word] = 0;	
		}
	}	
	pEndString += URI_FIRST_DATA_END_LENGTH;
	pLook4Word = pEndString;
	
	/* check if e-mail subject is present */
	if(!memcmp( pLook4Word, MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH))
	{		
		pEndString += MESSAGE_BEGIN_STRING_LENGTH;
		/* Retrieve message */
		memcpy( pSMSStruct->Message, pEndString, PayloadSize-(pEndString-pPayload));
		/* add end of string charactere */
		pSMSStruct->Message[PayloadSize-(pEndString-pPayload)] = 0;		
	}
}

/**
  * @brief  This fonction read the SMS and store data in a structure
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pSMSStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_ReadURI_SMS ( sRecordInfo *pRecordStruct, sSMSInfo *pSMSStruct )
{
	u8* pPayload;
	u32 PayloadSize;
	
	PayloadSize = ((u32)(pRecordStruct->PayloadLength3)<<24) | ((u32)(pRecordStruct->PayloadLength2)<<16) |
								((u32)(pRecordStruct->PayloadLength1)<<8)  | pRecordStruct->PayloadLength0;
	
	/* Read record header */
	pPayload = (u8*)(pRecordStruct->PayloadBufferAdd);
	
	if( pRecordStruct->NDEF_Type == URI_SMS_TYPE)
		NDEF_FillSMSStruct(pPayload , PayloadSize, pSMSStruct);
		
}

/**
  * @}
  */

/** @defgroup libSMS_Public_Functions
  * @{
  *	@brief  This file is used to manage SMS (stored or loaded in tag)
  */ 

/**
  * @brief  This fonction read NDEF and retrieve SMS information if any
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pSMSStruct : pointer on the structure to fill 
  * @retval SUCCESS : SMS information from NDEF have been retrieve
	* @retval ERROR : Not able to retrieve SMS information
  */
u16 NDEF_ReadSMS(sRecordInfo *pRecordStruct, sSMSInfo *pSMSStruct)
{
	u16 status = ERROR;
	uint16_t FileId;
	sRecordInfo *pSPRecordStruct;	
	u32 PayloadSize, RecordPosition;
	u8* pData;

	if( pRecordStruct->NDEF_Type == URI_SMS_TYPE )
	{	
		NDEF_ReadURI_SMS(pRecordStruct, pSMSStruct );
		status = SUCCESS;
	}
	else if( pRecordStruct->NDEF_Type == SMARTPOSTER_TYPE)
	{
		for (RecordPosition = 0; RecordPosition<pRecordStruct->NbOfRecordInSPPayload; RecordPosition++)
		{
			pSPRecordStruct = (sRecordInfo *)(pRecordStruct->SPRecordStructAdd[RecordPosition]);
			if(pSPRecordStruct->NDEF_Type == URI_SMS_TYPE )
			{
				NDEF_ReadURI_SMS(pSPRecordStruct, pSMSStruct );
				status = SUCCESS;
			}
			if(pSPRecordStruct->NDEF_Type == TEXT_TYPE )
			{
				PayloadSize = ((u32)(pSPRecordStruct->PayloadLength3)<<24) | ((u32)(pSPRecordStruct->PayloadLength2)<<16) |
										((u32)(pSPRecordStruct->PayloadLength1)<<8)  | pSPRecordStruct->PayloadLength0;
				
				/* The instruction content the UTF-8 language code that is not used here */
				pData = (u8*)pSPRecordStruct->PayloadBufferAdd;
				PayloadSize -= *pData+1; /* remove not usefull data */
				pData += *pData+1; /* set pointer on usefull data */
					
				memcpy(pSMSStruct->Information, pData, PayloadSize);
				/* add end of string charactere */
				pSMSStruct->Information[PayloadSize] = 0;		
			}
		}
	}
	
	CloseNDEFSession(FileId);
	
	return status;
}

/**
  * @brief  This fonction write the NDEF file with the SMS data given in the structure
	* @param	pSMSStruct : pointer on structure that contain the SMS information
  * @retval SUCCESS : the function is succesful
	* @retval ERROR : Not able to store NDEF file inside tag.
  */
u16 NDEF_WriteSMS ( sSMSInfo *pSMSStruct )
{
	u16 status = ERROR;
	u16 DataSize;
	u32 PayloadSP = 0, PayloadURI = 0, PayloadText = 0, Offset = 0;
	
	/* SMS is an URI but can be included in a smart poster to add text to give instruction to user for instance */
	
	/* SMS (smart poster) Record Header */
/************************************/	
/*	7 |  6 |  5 |  4 |  3 | 2  1  0 */
/*----------------------------------*/	
/* MB   ME   CF   SR   IL    TNF    */  /* <---- CF=0, IL=0 and SR=1 TNF=1 NFC Forum Well-known type*/
/*----------------------------------*/	
/*					TYPE LENGTH 						*/
/*----------------------------------*/
/*				PAYLOAD LENGTH 3 					*/	/* <---- Not Used  */
/*----------------------------------*/
/*			  PAYLOAD LENGTH 2 					*/  /* <---- Not Used  */
/*----------------------------------*/
/*				PAYLOAD LENGTH 1 					*/  /* <---- Not Used  */
/*----------------------------------*/	
/*				PAYLOAD LENGTH 0 					*/  
/*----------------------------------*/
/*					ID LENGTH 							*/  /* <---- Not Used  */
/*----------------------------------*/
/*							TYPE 								*/
/*----------------------------------*/
/*							 ID                 */  /* <---- Not Used  */ 
/************************************/
	
	/* SMS is a smart poster with URI record containing the phone number and Text record for SMS data */
	
	/* NDEF file must be written in 2 phases, first phase NDEF size is Null */
	NDEF_Buffer[NDEF_SIZE_OFFSET] = 0x00;
	NDEF_Buffer[NDEF_SIZE_OFFSET+1] = 0x00;
	
	/* we need to format it as Smart Poster */
	if( strlen(pSMSStruct->Information) != 0)
	{	
		/* fill smart poster record header */
		NDEF_Buffer[FIRST_RECORD_OFFSET] = 0xD1;   /* Record Flag */
		NDEF_Buffer[FIRST_RECORD_OFFSET+1] = SMART_POSTER_TYPE_STRING_LENGTH;
		NDEF_Buffer[FIRST_RECORD_OFFSET+2] = 0x00; /* Will be filled at the end when payload size is known */

		memcpy(&NDEF_Buffer[FIRST_RECORD_OFFSET+3], SMART_POSTER_TYPE_STRING, SMART_POSTER_TYPE_STRING_LENGTH);
	
	
		/* fill URI record header */
		Offset = FIRST_RECORD_OFFSET+3+SMART_POSTER_TYPE_STRING_LENGTH;
		NDEF_Buffer[Offset++] = 0x91;   /* Record Flag */
		NDEF_Buffer[Offset++] = URI_TYPE_STRING_LENGTH;
		NDEF_Buffer[Offset++] = 0x00; /* Will be filled at the end when payload size is known */

		memcpy(&NDEF_Buffer[Offset++], URI_TYPE_STRING, URI_TYPE_STRING_LENGTH);
	
		NDEF_Buffer[Offset] = 0x00; /* URI identifier no abbreviation */
	  PayloadURI = 1; /* URI identifier */
	
		/* fill URI payload */
		/* "sms:0629114591?body=Content of the sms" */
		memcpy(&NDEF_Buffer[Offset + PayloadURI], SMS_TYPE_STRING, SMS_TYPE_STRING_LENGTH);
		PayloadURI += SMS_TYPE_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pSMSStruct->PhoneNumber,strlen(pSMSStruct->PhoneNumber));
		PayloadURI += strlen(pSMSStruct->PhoneNumber);
		memcpy( &NDEF_Buffer[Offset + PayloadURI], URI_FIRST_DATA_END,URI_FIRST_DATA_END_LENGTH);
		PayloadURI += URI_FIRST_DATA_END_LENGTH;
		
		memcpy( &NDEF_Buffer[Offset + PayloadURI], MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH);
		PayloadURI += MESSAGE_BEGIN_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pSMSStruct->Message, strlen(pSMSStruct->Message));
		PayloadURI += strlen(pSMSStruct->Message);
		
		
		NDEF_Buffer[(Offset-2)] = (u8)PayloadURI;
	
		Offset = Offset + PayloadURI;
		/* fill Text record header */
		NDEF_Buffer[Offset++] = 0x51;   /* Record Flag */
		NDEF_Buffer[Offset++] = TEXT_TYPE_STRING_LENGTH;
		NDEF_Buffer[Offset++] = 0x00; /* Will be filled at the end when payload size is known */

		memcpy(&NDEF_Buffer[Offset], TEXT_TYPE_STRING, TEXT_TYPE_STRING_LENGTH);
		Offset += TEXT_TYPE_STRING_LENGTH;
		NDEF_Buffer[Offset] = strlen(ISO_ENGLISH_CODE_STRING); /* UTF-8 with x byte language code */
		PayloadText += 1; /* byte to specify number of byte to code language */
		memcpy(&NDEF_Buffer[Offset+PayloadText], ISO_ENGLISH_CODE_STRING, strlen(ISO_ENGLISH_CODE_STRING));
		PayloadText += strlen(ISO_ENGLISH_CODE_STRING);
	
	
		/* fill Text payload */
		/* "instruction to follow" */
		memcpy( &NDEF_Buffer[Offset+PayloadText], pSMSStruct->Information,strlen(pSMSStruct->Information));
		PayloadText += strlen(pSMSStruct->Information);
	
		NDEF_Buffer[(Offset-2)] = PayloadText;
	
	
		DataSize = Offset + PayloadText; /* Must not count the 2 byte that represent the NDEF size */
		PayloadSP = DataSize - (FIRST_RECORD_OFFSET+3+SMART_POSTER_TYPE_STRING_LENGTH);
	
	  /* SR payload must be less than 0xFF bytes */
		if(PayloadSP < 0x100)
			NDEF_Buffer[FIRST_RECORD_OFFSET+2] = (u8)PayloadSP;
		else
			return status;
	}
	/* it's a simple URI */
	else
	{
		/* fill URI record header */
		Offset = FIRST_RECORD_OFFSET;
		NDEF_Buffer[Offset++] = 0xD1;   /* Record Flag */
		NDEF_Buffer[Offset++] = URI_TYPE_STRING_LENGTH;
		NDEF_Buffer[Offset++] = 0x00; /* Will be filled at the end when payload size is known */

		memcpy(&NDEF_Buffer[Offset++], URI_TYPE_STRING, URI_TYPE_STRING_LENGTH);
	
		NDEF_Buffer[Offset] = 0x00; /* URI identifier no abbreviation */
	  PayloadURI = 1; /* URI identifier */
		
		/* fill URI payload */
		/* "sms:0629114591?body=Content of the sms" */
		memcpy(&NDEF_Buffer[Offset + PayloadURI], SMS_TYPE_STRING, SMS_TYPE_STRING_LENGTH);
		PayloadURI += SMS_TYPE_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pSMSStruct->PhoneNumber,strlen(pSMSStruct->PhoneNumber));
		PayloadURI += strlen(pSMSStruct->PhoneNumber);
		memcpy( &NDEF_Buffer[Offset + PayloadURI], MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH);
		PayloadURI += MESSAGE_BEGIN_STRING_LENGTH;
	
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pSMSStruct->Message, strlen(pSMSStruct->Message));
		PayloadURI += strlen(pSMSStruct->Message);
	
	
		NDEF_Buffer[(Offset-2)] = (u8)PayloadURI;
	
		DataSize = Offset + PayloadURI;
	}		
	
	/* Write NDEF */
	status = WriteData ( 0x00 , DataSize , NDEF_Buffer);
	
	/* Write NDEF size to complete*/
	if( status == NDEF_ACTION_COMPLETED)
	{
		DataSize -= 2; /* Must not count the 2 byte that represent the NDEF size */
		NDEF_Buffer[0] = (DataSize & 0xFF00)>>8;
		NDEF_Buffer[1] = (DataSize & 0x00FF);
	
		status = WriteData ( 0x00 , 2 , NDEF_Buffer);
	}
	
	if( status == NDEF_ACTION_COMPLETED)
		return SUCCESS;
	else
		return ERROR;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/


