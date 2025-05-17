typedef enum {
    SENSOR_BAROM = 1,
    SENSOR_ACC = 1 << 1
} enabled_sensors;

void initialize_system();

void start_flight();

void read_sensors();

void start_apogy();
