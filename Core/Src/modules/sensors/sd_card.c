#include "sd_card.h"
#include "communication.h"
#include <stdio.h>
#include <string.h>

#include "fatfs.h"

static FATFS fs_handle;

static bool is_enabled = false;

bool sd_card_is_enabled()
{
	return is_enabled;
}

void _sd_card_set_enabled()
{
	is_enabled = true;
}

sd_status sd_card_mount()
{
	FRESULT fs_status;

	char msg[256];

	//------------------[ Mount The SD Card ]--------------------
	fs_status = f_mount(&fs_handle, (const TCHAR *)_T(""), 1);

	if (fs_status != FR_OK)
	{
		sprintf(msg, "Error! While Mounting SD Card, Error Code: (%i)\r\n", fs_status);
		send_message(msg, PRIORITY_HIGH);
		return SD_ERR;
	}

	sprintf(msg, "SD Card Mounted Successfully! \r\n\n");
	send_message(msg, PRIORITY_HIGH);
	return SD_OK;
}

sd_status sd_card_get_info(sd_card_info* info)
{
	/*capacity related variable*/
	DWORD fre_clust;
	FATFS* fs_ptr = &fs_handle;
	FRESULT fs_status = f_getfree((const TCHAR *)_T(" "), &fre_clust, &fs_ptr);

	char msg[256];
	if (fs_status != FR_OK)
	{
		sprintf(msg, "Error! While reading free space, Error Code: (%i)\r\n", fs_status);
		send_message(msg, PRIORITY_HIGH);
	}

	info->total_space = (uint32_t)((fs_ptr->n_fatent - 2) * fs_ptr->csize * 0.5);
	sprintf(msg, "SD CARD Total Size: \t%lu\r\n", info->total_space);
	send_message(msg, PRIORITY_LOW);

	info->free_space = (uint32_t)(fre_clust * fs_ptr->csize * 0.5);

	sprintf(msg, "SD CARD Free Space: \t%lu\r\n", info->free_space);
	send_message(msg, PRIORITY_LOW);

	return SD_OK;
}

sd_status sd_card_open_file(sd_file* file_handle, char *filepath)
{
	// Open the file
	FRESULT fs_status = f_open(&file_handle->internal_handle, (const TCHAR *)filepath, FA_WRITE | FA_READ | FA_OPEN_APPEND);

	char msg[256];
	if (fs_status != FR_OK)
	{
		is_enabled = false;
		sprintf(msg, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", fs_status);
		send_message(msg, PRIORITY_HIGH);

		return SD_ERR;
	}

	return SD_OK;
}

sd_status sd_card_write(sd_file *file_handle, char* str)
{
	//f_puts((const TCHAR *)str, &file_handle->internal_handle);

	UINT WWC; // Read/Write Word Counter

	FRESULT fs_status = f_write(&file_handle->internal_handle, str, strlen(str), &WWC);

	if (fs_status != FR_OK)
	{
		char msg[256];
		sprintf(msg, "Error! While Writing To Text File, Error Code: (%i)\r\n", fs_status);
		send_message(msg, PRIORITY_HIGH);

		return SD_ERR;
	}

	return SD_OK;
}

sd_status sd_card_close(sd_file *file_handle)
{
	FRESULT fs_status = f_close(&file_handle->internal_handle);
	
	if (fs_status != FR_OK)
	{
		char msg[256];
		sprintf(msg, "Error! While Closing Text File, Error Code: (%i)\r\n", fs_status);
		send_message(msg, PRIORITY_HIGH);

		return SD_ERR;
	}

	return SD_OK;
}
