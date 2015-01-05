/**
  ******************************************************************************
  * @file    lib_NDEF_Email.c
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    20-November-2013
  * @brief   This file help to manage NDEF file that represent Email.
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
#include "lib_NDEF_Email.h"

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


/** @defgroup libEmail_Private_Functions
  * @{
  */


static void NDEF_FillEmailStruct( u8* pPayload, u32 PayloadSize, sEmailInfo *pEmailStruct);
static void NDEF_ReadURI_Email ( sRecordInfo *pRecordStruct, sEmailInfo *pEmailStruct );

/**
  * @brief  This fonction fill Email structure with information of NDEF message
	* @param	pPayload : pointer on the payload data of the NDEF message
	* @param	PayloadSize : number of data in the payload
	* @param	pEmailStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_FillEmailStruct( u8* pPayload, u32 PayloadSize, sEmailInfo *pEmailStruct)
{
	u8* pLastByteAdd, *pLook4Word, *pEndString ;
	u32 SizeOfKeyWord;
	
	/* First charactere force to NULL in case not matching found */
	*pEmailStruct->EmailAdd = 0;
	*pEmailStruct->Subject = 0;
	*pEmailStruct->Message = 0;
	
	/* Interresting information are stored before picture if any */
	/* Moreover picture is not used in this demonstration SW */	
	pLastByteAdd = (u8*)(pPayload + PayloadSize);
		
	/* first byte should be the "mailto:" well know URI type, skip it */
	pLook4Word = ++pPayload;
	
	/* Retrieve email add */
	if( pLook4Word != pLastByteAdd)
	{	
		pEndString = pLook4Word;
		while( memcmp( pEndString, URI_FIRST_DATA_END, URI_FIRST_DATA_END_LENGTH) && pEndString<pLastByteAdd )
		{
			pEndString++;
		}
		if( pEndString != pLastByteAdd)
		{
			memcpy( pEmailStruct->EmailAdd, pLook4Word, pEndString-pLook4Word);
			/* add end of string charactere */
			pEmailStruct->EmailAdd[pEndString-pLook4Word] = 0;	
		}
	}	
	
	pEndString += URI_FIRST_DATA_END_LENGTH;
	pLook4Word = pEndString;
	
	/* check if e-mail subject is present */
	if(!memcmp( pLook4Word, SUBJECT_BEGIN_STRING, SUBJECT_BEGIN_STRING_LENGTH))
	{		
		SizeOfKeyWord = SUBJECT_BEGIN_STRING_LENGTH;
	
		/* Retrieve subject */
		if( pLook4Word != pLastByteAdd)
		{
			pLook4Word += SizeOfKeyWord;
			pEndString = pLook4Word;
			while( memcmp( pEndString, URI_SECOND_DATA_END, URI_SECOND_DATA_END_LENGTH) && pEndString<pLastByteAdd )
			{
				pEndString++;
			}
			if( pEndString != pLastByteAdd)
			{
				memcpy( pEmailStruct->Subject, pLook4Word, pEndString-pLook4Word);
				/* add end of string charactere */
				pEmailStruct->Subject[pEndString-pLook4Word] = 0;	
			}
			pEndString += URI_SECOND_DATA_END_LENGTH;
		}		
	}

	pLook4Word = pEndString;	
	
	/* check if e-mail message is present */
	if(!memcmp( pLook4Word, MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH))
	{
		pEndString += MESSAGE_BEGIN_STRING_LENGTH;
		/* Retrieve message */
		memcpy( pEmailStruct->Message, pEndString, PayloadSize-(pEndString-pPayload+1));
		/* add end of string charactere */
		pEmailStruct->Message[PayloadSize-(pEndString-pPayload+1)] = 0;		
	}
}

/**
  * @brief  This fonction read the Email and store data in a structure
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pEmailStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_ReadURI_Email ( sRecordInfo *pRecordStruct, sEmailInfo *pEmailStruct )
{
	u8* pPayload;
	u32 PayloadSize;
	
	PayloadSize = ((u32)(pRecordStruct->PayloadLength3)<<24) | ((u32)(pRecordStruct->PayloadLength2)<<16) |
								((u32)(pRecordStruct->PayloadLength1)<<8)  | pRecordStruct->PayloadLength0;
	
	/* Read record header */
	pPayload = (u8*)(pRecordStruct->PayloadBufferAdd);
	
	if( pRecordStruct->NDEF_Type == URI_EMAIL_TYPE)
		NDEF_FillEmailStruct(pPayload , PayloadSize, pEmailStruct);
		
}

/**
  * @}
  */

/** @defgroup libEmail_Public_Functions
  * @{
  *	@brief  This file is used to manage Email (stored or loaded in tag)
  */ 

/**
  * @brief  This fonction read NDEF and retrieve Eamil information if any
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pEmailStruct : pointer on the structure to fill 
  * @retval SUCCESS : Email information from NDEF have been retrieved
	* @retval ERROR : Not able to retrieve Email information
  */
u16 NDEF_ReadEmail(sRecordInfo *pRecordStruct, sEmailInfo *pEmailStruct)
{
	u16 status = ERROR;
	sRecordInfo *pSPRecordStruct;	
	u32 PayloadSize, RecordPosition;
	u8* pData;
	

	if( pRecordStruct->NDEF_Type == URI_EMAIL_TYPE )
	{	
		NDEF_ReadURI_Email(pRecordStruct, pEmailStruct );
		status = SUCCESS;
	}
	else if( pRecordStruct->NDEF_Type == SMARTPOSTER_TYPE)
	{
		for (RecordPosition = 0; RecordPosition<pRecordStruct->NbOfRecordInSPPayload; RecordPosition++)
		{
			pSPRecordStruct = (sRecordInfo *)(pRecordStruct->SPRecordStructAdd[RecordPosition]);
			if(pSPRecordStruct->NDEF_Type == URI_EMAIL_TYPE )
			{
				NDEF_ReadURI_Email(pSPRecordStruct, pEmailStruct );
				status = SUCCESS;
			}
			if(pSPRecordStruct->NDEF_Type == TEXT_TYPE )
			{
				PayloadSize = ((u32)(pSPRecordStruct->PayloadLength3)<<24) | ((u32)(pSPRecordStruct->PayloadLength2)<<16) |
										((u32)(pSPRecordStruct->PayloadLength1)<<8)  | pSPRecordStruct->PayloadLength0;
				
				/* The instruction content the UTF-8 language code that is not used here */
				pData = (u8*)pSPRecordStruct->PayloadBufferAdd;
				pData += *pData+1;
					
				memcpy(pEmailStruct->Information, pData, PayloadSize);
			}
		}
	}
	
	return status;
}

/**
  * @brief  This fonction write the NDEF file with the Email data given in the structure
	* @param	pEmailStruct : pointer on structure that contain the Email information
  * @retval SUCCESS : the function is succesful
	* @retval ERROR : Not able to store NDEF file inside tag.
  */
u16 NDEF_WriteEmail ( sEmailInfo *pEmailStruct )
{
	u16 status = ERROR;
	u16 DataSize;
	u32 PayloadSP = 0, PayloadURI = 0, PayloadText = 0, Offset = 0;
	
	/* Email is an URI but can be included in a smart poster to add text to give instruction to user for instance */
	
	/* Email (smart poster) Record Header */
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
	
	/* Email is a smart poster with URI record containing the phone number and Text record for Email data */
	
	/* NDEF file must be written in 2 phases, first phase NDEF size is Null */
	NDEF_Buffer[NDEF_SIZE_OFFSET] = 0x00;
	NDEF_Buffer[NDEF_SIZE_OFFSET+1] = 0x00;
	
	/* we need to format it as Smart Poster */
	if( strlen(pEmailStruct->Information) != 0)
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
	
		NDEF_Buffer[Offset] = URI_ID_0x06; /* URI identifier mailto: */
	  PayloadURI = 1; /* URI identifier */
	
		/* fill URI payload */
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->EmailAdd,strlen(pEmailStruct->EmailAdd));
		PayloadURI += strlen(pEmailStruct->EmailAdd);	
		memcpy( &NDEF_Buffer[Offset + PayloadURI], URI_FIRST_DATA_END,URI_FIRST_DATA_END_LENGTH);
		PayloadURI += URI_FIRST_DATA_END_LENGTH;
				
		memcpy(&NDEF_Buffer[Offset + PayloadURI], SUBJECT_BEGIN_STRING, SUBJECT_BEGIN_STRING_LENGTH);
		PayloadURI += SUBJECT_BEGIN_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->Subject,strlen(pEmailStruct->Subject));
		PayloadURI += strlen(pEmailStruct->Subject);
		memcpy( &NDEF_Buffer[Offset + PayloadURI], URI_SECOND_DATA_END,URI_SECOND_DATA_END_LENGTH);
		PayloadURI += URI_SECOND_DATA_END_LENGTH;
		
		
		memcpy( &NDEF_Buffer[Offset + PayloadURI], MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH);
		PayloadURI += MESSAGE_BEGIN_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->Message, strlen(pEmailStruct->Message));
		PayloadURI += strlen(pEmailStruct->Message);
	
	
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
		/* "automatic warranty" */
		memcpy( &NDEF_Buffer[Offset+PayloadText], pEmailStruct->Information,strlen(pEmailStruct->Information));
		PayloadText += strlen(pEmailStruct->Information);
	
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
		/* "mailto:customer.service@st.com?subject=M24SR S/N 754FHFGJF46G329 WARRANTY&body=this is an auomatic warranty activation email" */
		memcpy(&NDEF_Buffer[Offset + PayloadURI], EMAIL_TYPE_STRING, EMAIL_TYPE_STRING_LENGTH);
		PayloadURI += EMAIL_TYPE_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->EmailAdd,strlen(pEmailStruct->EmailAdd));
		PayloadURI += strlen(pEmailStruct->EmailAdd);
		
		memcpy(&NDEF_Buffer[Offset + PayloadURI], SUBJECT_BEGIN_STRING, SUBJECT_BEGIN_STRING_LENGTH);
		PayloadURI += SUBJECT_BEGIN_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->Subject,strlen(pEmailStruct->Subject));
		PayloadURI += strlen(pEmailStruct->Subject);
		
		memcpy( &NDEF_Buffer[Offset + PayloadURI], MESSAGE_BEGIN_STRING, MESSAGE_BEGIN_STRING_LENGTH);
		PayloadURI += MESSAGE_BEGIN_STRING_LENGTH;
		memcpy( &NDEF_Buffer[Offset + PayloadURI], pEmailStruct->Message, strlen(pEmailStruct->Message));
		PayloadURI += strlen(pEmailStruct->Message);
	
	
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


