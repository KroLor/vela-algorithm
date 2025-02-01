short check_acc_identity();
short acc_power_on();

/// @brief Возвращает ускорение по трем осям
/// @param xyz массив из трех значений, в котором будет хранится ускорение по осям x,y,z соответственно
void read_acceleration_xyz(double* buffer_xyz);