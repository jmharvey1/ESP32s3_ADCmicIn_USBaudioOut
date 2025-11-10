#ifndef NVS_SUPRT_H
#define NVS_SUPRT_H
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <nvs_flash.h>
#include <nvs.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// Return codes
typedef enum {
    NVS_OK = 0,
    NVS_INIT_ERROR,
    NVS_OPEN_ERROR,
    NVS_READ_ERROR,
    NVS_WRITE_ERROR
} nvs_result_t;

extern int AudioOutMode; //0=I2S PDM, 1=USB Audio
class NVS_Suprt {
public:
    NVS_Suprt() = default;
    ~NVS_Suprt() = default;

    // Initialize NVS
    nvs_result_t nvs_init(void);

    // Read/Write functions for different types
    template <class T>
    nvs_result_t nvs_read_val(const char *key, T &value);
    nvs_result_t nvs_write_val(const char *key, int value);

    nvs_result_t nvs_read_string(const char *key, char *value);
    nvs_result_t nvs_write_string(const char *key, char *value);

    // Clear all stored values
    nvs_result_t nvs_clear_all(void);

    nvs_result_t get_AudioOutMode(int &mode);

    void SaveUsrVals(void);

private:
    // Add private members here if needed (e.g., nvs_handle_t handle;)
};

// #ifdef __cplusplus
// }
// #endif

#endif // NVS_SUPRT_H