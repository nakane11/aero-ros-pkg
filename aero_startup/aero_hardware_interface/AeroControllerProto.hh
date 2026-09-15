#ifndef AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_
#define AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <unordered_map>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "aero_hardware_interface/CommandList.hh"
#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AJointIndex.hh"

using namespace boost::asio;

namespace aero
{
  namespace controller
  {
    /// @brief SEED controller via USB/RS485
    class SEED485Controller
    {
      /// @brief constructor
      /// @param _port USB port file name
      /// @param _id CAN bus ID
     public: SEED485Controller(const std::string& _port, uint8_t _id);

      /// @brief destructor
     public: ~SEED485Controller();

      /// @brief get version of SEED controller
     public: std::string get_version();

      /// @brief get voltage of SEED controller
     public: float get_voltage();

      /// @brief read from SEED controller
     public: void read(std::vector<uint8_t>& _read_data, const size_t _length=RAW_DATA_LENGTH);

      /// @brief read_some bounded by a timeout.
      ///   boost::asio::serial_port keeps its fd non-blocking
      ///   internally, so a synchronous read_some() with no data
      ///   available never blocks; without an explicit deadline this
      ///   loops effectively forever when nothing is arriving. Uses
      ///   poll() on the raw fd for a real wall-clock timeout.
      /// @return number of bytes read, 0 on timeout
     private: int read_some_timed(std::vector<uint8_t>& _buf, size_t _want,
                                  int _timeout_ms);

      /// @brief send command to SEED controller
      /// @param _cmd Command ID
      /// @param _time Destination time
      /// @param _send_data data buffer
     public: void send_command(uint8_t _cmd, uint16_t _time,
                               std::vector<uint8_t>& _send_data);

      /// @brief send single command to SEED controller
      /// @param _cmd Command ID
      /// @param _num Send ID
      /// @param _data data
     public: void send_command(uint8_t _cmd, uint8_t _num, uint16_t _data);

     public: void send_command(uint8_t _cmd, uint8_t _sub, uint16_t _time,
                               std::vector<uint8_t>& _send_data);

      /// @brief send_executing script command
     public: void AERO_Snd_Script(uint16_t sendnum, uint8_t scriptnum);

      /// @brief flush io buffer
     public: void flush();

      /// @brief send raw data to SEED controller
      /// @param _send_data raw data buffer
     public: void send_data(std::vector<uint8_t>& _send_data);

      /// @brief set / unset verbose mode
      /// @param val verbose mode
     public: void verbose(bool val) {verbose_ = val;}
      /// @brief get verbose mode flag
      /// @return verbose mode flag
     public: bool verbose() {return verbose_;}

      /// @brief getdebug mode flag
      /// @return true if in debug mode
     public: bool is_debug_mode() {return !ser_.is_open();}

     private: io_service io_;

     private: serial_port ser_;

     private: uint8_t id_;

     private: bool verbose_;

     private: boost::mutex mtx_;
    };  // SEED485Controller

    /// @brief super class of body controller,
    /// has SEED485Controller and some SEED command fucntions.
    class AeroControllerProto
    {
      /// @brief constructor
      /// @param _port USB port file name
      /// @param _id CAN bus ID
     public: AeroControllerProto(const std::string& _port, uint8_t _id);

      /// @brief destructor
     public: ~AeroControllerProto();

      /// @brief get version of SEED controller
     public: std::string get_version();

      /// @brief get voltage of SEED controller
     public: float get_voltage();

      /// @brief servo on command
     public: void servo_on();

      /// @brief servo off command
     public: void servo_off();

      /// @brief servo toggle command
      /// @param _d0 1: on, 0: off
     protected: void servo_command(int16_t _d0);

     public: std::vector<int16_t> get_reference_stroke_vector();

     public: std::vector<int16_t> get_actual_stroke_vector();

     public: std::vector<int16_t> get_status_vec();

     public: std::string get_stroke_joint_name(size_t _idx);

     public: int get_number_of_angle_joints();

     public: int get_number_of_strokes();

     public: int32_t get_ordered_angle_id(std::string _name);

     public: bool get_joint_name(int32_t _joint_id, std::string &_name);

     public: bool get_status();

     public: bool get_status(std::vector<bool>& _status_vector);

      /// @brief get current position from seed_
      ///   to access position externally, use get_actual_stroke_vector
     public: void update_position();

      /// @brief true once after communication recovered from a loss
      ///   (e.g. the servo-on button was cycled). Reading it clears
      ///   the flag, so the caller can restart its controllers exactly
      ///   once per recovery.
     public: bool check_comm_recovered();

      /// @brief send one STGET (CMD_WATCH_MISSTEP) and debounce the
      ///   result into an origin-return (calibration) completion
      ///   signal. The motor driver simply does not reply to STGET
      ///   while mid origin-return and replies reliably once it is
      ///   done, regardless of what the Robot Status bits then say --
      ///   see the comment on calibration_ok_streak_ for why the bits
      ///   themselves are not used for this judgement.
      /// @return true once kCalibrationStreakThreshold consecutive
      ///   calls got a reply
     public: bool poll_calibration_status();

      /// @brief track communication loss/recovery of a read-back
     protected: void note_comm_result(bool _ok);

      /// @brief updates robot status (checks step out joints)
     public: void update_status();

     public: void reset_status();

      /// @brief send Get_Cur command
      /// @param _stroke_vector stroke vector
     public: void get_current(std::vector<int16_t>& _stroke_vector);

      /// @brief send Get_Tmp command
      /// @param _stroke_vector stroke vector
     public: void get_temperature(std::vector<int16_t>& _stroke_vector);

      /// @brief get data from buffer,
      ///   this does not call command, but only read from buffer
      /// @param _stroke_vector stroke vector
      /// @return true if a valid, recognized response was parsed
     protected: bool get_data(std::vector<int16_t>& _stroke_vector);

      /// @brief abstract of get commands. Resends the command a few
      ///   times if no valid response comes back -- a command sent
      ///   while the motor driver isn't listening yet (e.g. mid
      ///   power-on calibration) is otherwise lost for good, since
      ///   nothing else would ever resend it.
      /// @param _cmd command id
      /// @param _stroke_vector stroke vector
      /// @return true if a valid response was eventually received
     protected: bool get_command(uint8_t _cmd,
                                 std::vector<int16_t>& _stroke_vector);

     protected: bool get_command(uint8_t _cmd, uint8_t _sub,
                                 std::vector<int16_t>& _stroke_vector);

      /// @brief set position command (waiting return of current position)
      /// @param _stroke_vector stroke vector, MUST be DOF bytes
      /// @param _time time[ms]
     public: void set_position(std::vector<int16_t>& _stroke_vector,
                               uint16_t _time);

      /// @brief set position command (no wait)
      /// @param _stroke_vector stroke vector, MUST be DOF bytes
      /// @param _time time[ms]
     public: void set_position_no_wait(std::vector<int16_t>& _stroke_vector,
                               uint16_t _time);

      /// @brief send Motor_Cur command
      /// @param _stroke_vector stroke vector
     public: void set_max_current(std::vector<int16_t>& _stroke_vector);

      /// @brief send Motor_Cur command
      /// @param _num Send id
      /// @param _dat data
     public: void set_max_single_current(int8_t _num, int16_t _dat);

      /// @brief send Motor_Acc command
      /// @param _stroke_vector stroke vector
     public: void set_accel_rate(std::vector<int16_t>& _stroke_vector);

      /// @brief send Motor_Gain command
      /// @param _stroke_vector stroke vector
     public: void set_motor_gain(std::vector<int16_t>& _stroke_vector);

      /// @brief send single command to SEED controller
      /// @param _cmd Command ID
      /// @param _num Send ID
      /// @param _data data
     public: void set_command(uint8_t _cmd, uint8_t _num, uint16_t _data);

      /// @brief abstract of set commands
      /// @param _cmd command id
      /// @param _stroke_vector stroke vector
     protected: void set_command(uint8_t _cmd,
                                 std::vector<int16_t>& _stroke_vector);

      /// @brief stoke_vector to raw command bytes
     protected: void stroke_to_raw_(std::vector<int16_t>& _stroke,
                                    std::vector<uint8_t>& _raw);

     protected: bool verbose_;

     protected: boost::mutex ctrl_mtx_;

     protected: SEED485Controller seed_;

     protected: std::vector<int16_t> stroke_vector_;

     protected: std::vector<int16_t> stroke_ref_vector_;

     protected: std::vector<int16_t> stroke_cur_vector_;

      /// @brief true once a real communication loss is confirmed
      ///   (kLossStreakThreshold_ consecutive failed read-backs), not
      ///   on a single failure -- a lone dropped/misaligned byte is a
      ///   routine, self-recovering glitch and must not be treated the
      ///   same as the motor driver actually losing power.
     protected: bool comm_was_lost_;

      /// @brief set when a real recovery is confirmed
      ///   (kRecoveryStreakThreshold_ consecutive good read-backs
      ///   after a loss), cleared by check_comm_recovered(). Debounced
      ///   the same way as comm_was_lost_: a single lucky read in the
      ///   middle of noisy comms must not trigger a full controller
      ///   restart.
     protected: bool comm_recovered_latch_;

      /// @brief consecutive failed / good read-backs since the last
      ///   state change, used to debounce comm_was_lost_ /
      ///   comm_recovered_latch_.
     protected: int comm_fail_streak_;
     protected: int comm_ok_streak_;

      /// @brief consecutive successful STGET replies since the last
      ///   failure, used to debounce origin-return (calibration)
      ///   completion in poll_calibration_status(). Robot Status bits
      ///   (see get_data()) are a live summary the driver keeps
      ///   updated from its own background polling of the motor
      ///   drivers, not a one-shot "origin-return finished" event --
      ///   e.g. the "motor abnormal" bit stays set for as long as the
      ///   servo itself is off, long after a successful calibration --
      ///   so they must not be used to judge calibration completion.
     protected: int calibration_ok_streak_;

     protected: std::vector<AJointIndex> stroke_joint_indices_;

     protected: std::vector<int16_t> status_vector_;

     protected: bool bad_status_;

     protected:
      std::unordered_map<std::string, int32_t> angle_joint_indices_;
    };  // AeroControllerProto

  /////////////////////
  // nonclass functions
  /////////////////////

  /// @brief decode short(int16_t) from byte(uint8_t)
  int16_t decode_short_(uint8_t* _raw);

  /// @brief ecnode short(int16_t) to byte(uint8_t)
  void encode_short_(int16_t _value, uint8_t* _raw);
  }
}

#endif
