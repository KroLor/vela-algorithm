typedef enum {
    SENSOR_BAROM = 1,
    SENSOR_ACC = 1 << 1
} enabled_sensors;

typedef enum {
    RESCUE_SYSTEM_OK,
    RESCUE_SYSTEM_FAIL
} res_sys_status;

typedef enum {
    APOGY_YES,
    APOGY_NO
} apogy_status;

typedef enum {
    LANDING_YES,
    LANDING_NO
} landing_status;

void initialize_system();

void start_flight();

void read_sensors();

void start_apogy();

res_sys_status check_res_sys();
apogy_status check_apogy();
landing_status check_landing();