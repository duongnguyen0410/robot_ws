#ifndef PLCComms_HPP
#define PLCComms_HPP

#include <modbus/modbus.h>
#include <iostream>

class PLCComms
{
public:
  PLCComms() : modbus_ctx_(nullptr) {}

  bool connect(const std::string& ip_address, int port)
  {
    modbus_ctx_ = modbus_new_tcp(ip_address.c_str(), port);
    if (modbus_ctx_ == nullptr)
    {
      std::cerr << "Failed to create Modbus context." << std::endl;
      return false;
    }

    if (modbus_connect(modbus_ctx_) == -1)
    {
      std::cerr << "Failed to connect to the PLC." << std::endl;
      modbus_free(modbus_ctx_);
      modbus_ctx_ = nullptr;
      return false;
    }

    return true;
  }

  void disconnect()
  {
    if (modbus_ctx_ != nullptr)
    {
      modbus_close(modbus_ctx_);
      modbus_free(modbus_ctx_);
      modbus_ctx_ = nullptr;
    }
  }

  bool connected() const
  {
    return (modbus_ctx_ != nullptr);
  }

  bool set_motor_values(int16_t motor_r, int16_t motor_l)
  {
    uint16_t data[] = {motor_r, motor_l};
    const uint16_t motor_addr[] = {70, 80};
    const int length = sizeof(data)/ sizeof(data[0]);

    for (int i = 0; i < length; ++i)
    {
      if (modbus_write_registers(modbus_ctx_, motor_addr[i], 1, &data[i]) == -1)
      {
        std::cerr << "Failed to write data to register " << motor_addr[i] << std::endl;
        return false;
      }
    }
    std::cout << "cmd_l_vel: "<< motor_l << ", cmd_r_vel: "<< motor_r << std::endl;
    std::cout << "Data written to PLC successfully." << std::endl;
    
    return true;
  }

  bool read_encoder_values(int64_t &val_l, int64_t &val_r)
  {
    const int16_t encoder_addr[] = {130, 110};
    const int read_len = sizeof(int16_t) / sizeof(int16_t);

    int16_t prev_enc_val = 0;

    int16_t cur_val_r = 0;
    int cur_val_l = 0;

    for (int i = 0; i < 2; ++i)
    {
      int16_t encoder_value;
      int num_read = modbus_read_registers(modbus_ctx_, encoder_addr[i], read_len, reinterpret_cast<uint16_t*>(&encoder_value));

      int16_t current_value_r, current_value_l;

      if (num_read == -1)
      {
        std::cerr << "Failed to read encoder from the PLC." << std::endl;
        return false;
      }

      if (num_read != read_len)
      {
        std::cerr << "Unexpected number of encoder registers read from the PLC." << std::endl;
        return false;
      }

      if (i == 0){
        encoder_accumulator_r_ += calculate_encoder_change(previous_encoder_value_r_, encoder_value);
        previous_encoder_value_r_ = encoder_value;
        val_r = -encoder_accumulator_r_;
        // val_r = -calculate_actual_velocity(encoder_accumulator_r_);
      }
      
      else if (i == 1){
        encoder_accumulator_l_ += calculate_encoder_change(previous_encoder_value_l_, encoder_value);
        previous_encoder_value_l_ = encoder_value;
        val_l = encoder_accumulator_l_;
        // val_l = calculate_actual_velocity(encoder_accumulator_l_);
      }
    }
    std::cout << "enc_l_motor: "<< val_l << ", enc_r_motor: "<< val_r << std::endl;

    return true;
  }

private:
  modbus_t* modbus_ctx_;
  int64_t encoder_accumulator_r_ = 0;
  int64_t encoder_accumulator_l_ = 0;
  int16_t previous_encoder_value_r_ = 0;
  int16_t previous_encoder_value_l_ = 0;

  int64_t calculate_encoder_change(int16_t previous_value, int16_t current_value)
  {
    int64_t delta = current_value - previous_value;
    if (delta > 32767) 
      delta -= 65536;
    else if (delta < -32767)
      delta += 65536;
    return delta;
  }
  
  int16_t calculate_actual_velocity(int16_t cur_enc_value)
  {
    int16_t er_per_wr = 3000;
    int16_t d = 20;
    double wr_per_er = 0.021;
    return (cur_enc_value * wr_per_er) / 0.5;
  }
};

#endif // PLCComms_HPP
