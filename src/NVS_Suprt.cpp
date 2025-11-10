/*20251109 created to support saving user setting(s) to NonVolitle Storage (NVS)*/
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "Text2Dsply.h"
#include "NVS_Suprt.h"
#include "nvs_handle.hpp"
//#include <cmath>
//#include <cstring>
esp_err_t ret;

nvs_result_t NVS_Suprt::nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    return NVS_OK;
}

nvs_result_t NVS_Suprt::nvs_write_val(const char *key, int value)
/* write numeric data to NVS; If the data is "stored",return with an exit status of "1" */
{
  nvs_result_t stat = NVS_WRITE_ERROR;
  char TxtBuf[50];
  esp_err_t ret = ESP_OK;
  // const uint16_t* p = (const uint16_t*)(const void*)&value;
  //  Handle will automatically close when going out of scope or when it's reset.
  std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
  if (ret != ESP_OK)
  {
    sprintf(TxtBuf, "Error (%s) opening WRITE NVS value handle for: %s!\n", esp_err_to_name(ret), key);
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
  }
  else
  {
    ret = handle->set_item(key, value);
    switch (ret)
    {
    case ESP_OK:
      ret = handle->commit();
      if (ret != ESP_OK)
      {
        sprintf(TxtBuf, "Commit Failed (%s) on %s!\n", esp_err_to_name(ret), key);
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
      }
      else
        /* exit point when everything works as it should */
        stat = NVS_OK;
      break;
    default:
      sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
      if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
      //delay(3000);
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }
  return stat;
}
////////////////////////////////////////////
/*Save data to NVS memory routines*/

nvs_result_t NVS_Suprt::nvs_write_string(const char *key, char *value)
/* write string data to NVS */
{
  char TxtBuf[50];  
  nvs_result_t stat = NVS_WRITE_ERROR;
  esp_err_t ret = ESP_OK;
  //  Handle will automatically close when going out of scope or when it's reset.
  std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
  if (ret != ESP_OK)
  {
    // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    sprintf(TxtBuf, "Error (%s) opening WRITE NVS string handle for: %s!\n", esp_err_to_name(ret), key);
    if (Txt2Dsply_mutx != NULL)
    {
      if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
      {
        Print2Dsply(TxtBuf);
        xSemaphoreGive(Txt2Dsply_mutx);
      }
    }
    else
    {
      printf("%s", TxtBuf);
    }
   
  }
  else
  {

    ret = handle->set_string(key, value);
    switch (ret)
    {
    case ESP_OK:
      /* write operation worked; go ahead and /lock the data in */
      ret = handle->commit();
      if (ret != ESP_OK)
      {
        sprintf(TxtBuf, "Commit Failed (%s) on %s!\n", esp_err_to_name(ret), key);
        if (Txt2Dsply_mutx != NULL)
        {
          if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
          {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
          }
        }
        else
        {
          printf("%s", TxtBuf);
        }
       
      }
      else
      {
        /* exit point when everything works as it should */
        stat = NVS_OK;
        break;
      }
    default:
      sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
      if (Txt2Dsply_mutx != NULL)
      {
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
          Print2Dsply(TxtBuf);
          xSemaphoreGive(Txt2Dsply_mutx);
        }
      }
      else
      {
        printf("%s", TxtBuf);
      }
      //delay(3000);
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }
  return stat;
}
/////////////////////////////////////////////////////////////
template <class T>
nvs_result_t NVS_Suprt::nvs_read_val(const char *key, T &value)
/* read numeric data from NVS; If the data is "found",return with an exit status of "1" */
{
  nvs_result_t stat = NVS_READ_ERROR;
  char TxtBuf[60];
  esp_err_t ret = ESP_OK;
  const char* namespace_name = "storage"; 
  nvs_handle_t handle;
  //ret = nvs_open_from_partition("custom_nvs_partition", namespace_name, NVS_READWRITE, &handle);
  ret = nvs_open(namespace_name, NVS_READONLY, &handle);
  //  Handle will automatically close when going out of scope or when it's reset.
  //std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READONLY, &ret);
  // std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle_from_partition("nvs", //partition_name
  //        "nvs", //ns_name
  //        NVS_READONLY,
  //        &ret);
  
  printf("nvs_read_val: opened nvs handle for key=%s; ret = %d\n", key, (int)ret);
  if (ret != ESP_OK)
  {
    sprintf(TxtBuf, "Error (%s) opening 'nvs_read_val' handle for: %s!\n", esp_err_to_name(ret), key);
    if (Txt2Dsply_mutx != NULL)
    {
      if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
      {
        Print2Dsply(TxtBuf);
        xSemaphoreGive(Txt2Dsply_mutx);
      }
    }
    else
    {
      printf("%s", TxtBuf);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return stat;
    }
  else
  {
    //ret = handle->get_item(key, value);
    ret = nvs_get_i32(handle, key, reinterpret_cast<int32_t*>(&value));
    switch (ret)
    {
    case ESP_OK:
      /* exit point when everything works as it should */
      //printf("%s = %" PRIu32 "\n", key, static_cast<uint32_t>(value));
      stat = NVS_OK;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      sprintf(TxtBuf, "The value for %s is not initialized yet!\n", key);
      if (Txt2Dsply_mutx != NULL)
      {
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
          Print2Dsply(TxtBuf);
          xSemaphoreGive(Txt2Dsply_mutx);
        }
      }
      else
      {
        printf("%s", TxtBuf);
      }
      break;
    default:
      sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
      printf("%s", TxtBuf);
      if (Txt2Dsply_mutx != NULL)
      {
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
          Print2Dsply(TxtBuf);
          xSemaphoreGive(Txt2Dsply_mutx);
        }
      }
      else
      {
        printf("%s", TxtBuf);
      }
      //delay(3000);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    
  }
  return stat;
}
/////////////////////////////////////////////////////
/* the following code handles the read from & write to NVS processes needed to handle the User CW prefences/settings  */

nvs_result_t NVS_Suprt::nvs_read_string(const char *key, char *value)
/* read string data from NVS*/
{
  nvs_result_t stat = NVS_READ_ERROR;
  char TxtBuf[50];
  esp_err_t ret = ESP_OK;
  // const uint16_t* p = (const uint16_t*)(const void*)&value;
  //  Handle will automatically close when going out of scope or when it's reset.
  std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
  if (ret != ESP_OK)
  {
    sprintf(TxtBuf, "Error (%s) opening READ NVS string handle for: %s!\n", esp_err_to_name(ret), key);
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
  }
  else
  {
    size_t required_size;
    ret = handle->get_item_size(nvs::ItemType::SZ, key, required_size);
    switch (ret)
    {
    case ESP_OK:

      if (required_size > 0 && required_size < 100)
      {
          char temp[100];
          ret = handle->get_string(key, temp, required_size);
          int i;
          for (i = 0; i < 100; i++)
          {
              value[i] = temp[i];
              if (temp[i] == 0)
                  break;
          }

          sprintf(TxtBuf, "%d characters copied to %s\n", i, key);
          if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
          {
              Print2Dsply(TxtBuf);
              xSemaphoreGive(Txt2Dsply_mutx);
          }

          stat = NVS_OK;
      }
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      /*TODO could need support code here*/
      break;
    default:
      sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
      if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
      //delay(3000);
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }
  return stat;
}
/////////////////////////////////////////////////////////////
nvs_result_t NVS_Suprt::get_AudioOutMode(int &mode)
{
  nvs_result_t Rstat;
  Rstat = nvs_read_val("AudioOutMode", mode);
  return Rstat; 
};
/////////////////////////////////////////////////////////////
void NVS_Suprt::SaveUsrVals(void)
{
	/* Unlock the Flash Program Erase controller */
	char buf[30];
	bool GudFlg = true;
	char call[10];
	char mem[80];
	int i;
	nvs_result_t Rstat;
	bool zeroFlg = false;
	/*Save Debug Setting*/
	Rstat = nvs_write_val("AudioOutMode", AudioOutMode);
	if (Rstat != NVS_OK)
		GudFlg = false;
	/* Save current Decoder NiteMode value */
	
	if (GudFlg)
  {
    sprintf(buf, "User Params SAVED\n");
    
  }
  else
	{
		sprintf(buf, "SAVE FAILED\n");
		
	}
  if (Txt2Dsply_mutx != NULL)
  {
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
    {
      Print2Dsply(buf);
      xSemaphoreGive(Txt2Dsply_mutx);
    }
  }
  else
  {
    printf("%s", buf);
  }
}
////////////////////////////////////////////////////////////