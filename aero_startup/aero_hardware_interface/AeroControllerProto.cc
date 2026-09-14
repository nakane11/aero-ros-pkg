#include "AeroControllerProto.hh"
#include <chrono>
#include <poll.h>

using namespace boost::asio;
using namespace aero;
using namespace controller;

namespace {
  // Per-attempt read timeout for a single poll()+read_some(). The
  // control loop runs at 15 Hz (aero_bringup.launch controller_rate),
  // i.e. a ~67 ms period, so this must stay well under that -- a real
  // response, once the driver is listening, arrives within a handful
  // of milliseconds. This only bounds how long we wait for bytes that
  // may never come; it previously sat at 100 ms, already longer than
  // one whole control period by itself.
  const int kReadTimeoutMs = 15;
  // Total per-phase budget (sync-to-header / read-body), in units of
  // kReadTimeoutMs. Kept short too -- long-run retrying is handled one
  // level up, by get_command() resending the whole command.
  const int kMaxSyncRetries = 5;
  const int kMaxBodyReadAttempts = 5;

  // How many times get_command() resends the command before giving
  // up. This is the actual fix for the original bug: a command sent
  // while the motor driver is mid-calibration (servo-on button just
  // pressed) and not yet listening is silently lost on the wire, and
  // nothing was ever resending it -- the code would just wait forever
  // for a reply to a request the hardware never received. Kept small:
  // at kReadTimeoutMs * (kMaxSyncRetries + kMaxBodyReadAttempts) per
  // attempt, this is already several control periods worst-case, and
  // it used to be more than an order of magnitude higher (8 attempts
  // at 100 ms/try).
  const int kMaxCommandRetries = 3;

  // How many consecutive failed / good read-backs are required before
  // comm_was_lost_ / comm_recovered_latch_ actually flip. A single
  // failed or single lucky read is a routine, self-recovering glitch
  // (e.g. one misaligned byte from USB-serial jitter) and must not be
  // confused with the motor driver truly losing/regaining power --
  // that confusion previously made a single stray read failure look
  // exactly like a RUNSTOP power cycle, triggering a full controller
  // restart (stop/start + settle wait) during otherwise normal
  // operation. 10 consecutive cycles is ~0.7 s at the 15 Hz control
  // rate, comfortably longer than a one-off glitch but far shorter
  // than an actual servo power cycle.
  const int kLossStreakThreshold = 10;
  const int kRecoveryStreakThreshold = 10;

  // Prints _msg through std::cerr at most once every kLogThrottleSec,
  // regardless of how often this is called. Communication-loss errors
  // repeat every retry, which would otherwise flood the console.
  const double kLogThrottleSec = 10.0;
  void log_throttled(const std::string& _msg,
                      std::chrono::steady_clock::time_point& _last)
  {
    auto now = std::chrono::steady_clock::now();
    if (_last.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - _last).count() < kLogThrottleSec) {
      return;
    }
    _last = now;
    std::cerr << _msg << std::endl;
  }
}  // namespace

//////////////////////////////////////////////////
int SEED485Controller::read_some_timed(
    std::vector<uint8_t>& _buf, size_t _want, int _timeout_ms)
{
  // boost::asio::serial_port keeps its fd non-blocking internally, so
  // a synchronous read_some() with no data available never blocks. A
  // plain retries<N loop around it (as this file used to have) is
  // therefore effectively unbounded in wall-clock time when nothing is
  // arriving. poll() on the raw fd gives a real, wall-clock timeout
  // without touching asio's own async machinery.
  int fd =
#if ((BOOST_VERSION / 100 % 1000) > 50)
    ser_.lowest_layer().native_handle();
#else
    ser_.lowest_layer().native();
#endif

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(_timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return 0;
    }
    int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now).count());

    pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int rc = ::poll(&pfd, 1, remaining_ms);
    if (rc <= 0 || !(pfd.revents & POLLIN)) {
      return 0;  // timed out, or nothing readable yet
    }

    boost::system::error_code ec;
    size_t n = ser_.read_some(buffer(_buf, _want), ec);
    if (!ec) {
      return static_cast<int>(n);
    }
    if (ec == boost::asio::error::would_block ||
        ec == boost::asio::error::try_again) {
      // poll() said readable, but the non-blocking read raced and
      // found nothing yet -- not a real timeout, retry within the
      // same deadline instead of burning one of the caller's attempts.
      continue;
    }
    return 0;  // genuine read error
  }
}

//////////////////////////////////////////////////
SEED485Controller::SEED485Controller(
    const std::string& _port, uint8_t _id) :
  ser_(io_), verbose_(false), id_(_id)
{
  if (_port == "") {
    std::cerr << "empty serial port name: entering debug mode...\n";
    // verbose_ = true;
    return;
  }

  boost::system::error_code err;

  ser_.open(_port, err);
  usleep(1000 * 1000);

  if (err) {
    std::cerr << "could not open " << _port << "\n";
    // verbose_ = true;
    return;
  }

  try {
    ser_.set_option(serial_port_base::baud_rate(1000000));
//     struct termios tio;
// #if ((BOOST_VERSION / 100 % 1000) > 50)
//     ::tcgetattr(ser_.lowest_layer().native_handle(), &tio);
//     ::cfsetospeed(&tio, 1000000);
//     ::cfsetispeed(&tio, 1000000);
//     ::tcsetattr(ser_.lowest_layer().native_handle(), TCSANOW, &tio);
// #else  // 12.04
//     ::tcgetattr(ser_.lowest_layer().native(), &tio);
//     ::cfsetospeed(&tio, 1000000);
//     ::cfsetispeed(&tio, 1000000);
//     ::tcsetattr(ser_.lowest_layer().native(), TCSANOW, &tio);
// #endif
  } catch (std::exception& e) {
    std::cerr << "baudrate: " << e.what() << "\n";
  }

  try {
    ser_.set_option(serial_port_base::character_size(
	serial_port_base::character_size(8)));
    ser_.set_option(serial_port_base::flow_control(
        serial_port_base::flow_control::none));
    ser_.set_option(serial_port_base::parity(
        serial_port_base::parity::none));
    ser_.set_option(serial_port_base::stop_bits(
        serial_port_base::stop_bits::one));
  } catch (std::exception& e) {
    std::cerr << e.what() << "\n";
  }
}

//////////////////////////////////////////////////
SEED485Controller::~SEED485Controller()
{
  if (ser_.is_open()) ser_.close();
}

//////////////////////////////////////////////////
std::string SEED485Controller::get_version()
{
  std::vector<uint8_t> data(6);
  data[0] = 0xFD;
  data[1] = 0xDF;
  data[2] = 0x02;
  data[3] = CMD_GET_VERSION;
  data[4] = 0x00;

  // check sum
  int32_t b_check_sum = 0;

  for (size_t i = 2; i < data.size() - 1; ++i) {
    b_check_sum += data[i];
  }

  data[data.size() - 1] =
    ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);

  send_data(data);

  std::vector<uint8_t> dat;
  dat.resize(11);
  read(dat, 11);
  uint16_t header = decode_short_(&dat[0]);
  int16_t cmd;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&cmd);
  bvalue[0] = dat[3];
  bvalue[1] = 0x00;

  if (header != 0xdffd || cmd != CMD_GET_VERSION) {
    flush();
    std::cerr << "Proto: ERROR: invalid header" << std::endl;
    return "";
  }

  uint8_t dat1 = dat[RAW_HEADER_OFFSET];
  uint8_t dat2 = dat[RAW_HEADER_OFFSET + 1];
  uint8_t dat3 = dat[RAW_HEADER_OFFSET + 2];
  uint8_t dat4 = dat[RAW_HEADER_OFFSET + 3];
  uint8_t dat5 = dat[RAW_HEADER_OFFSET + 4];

  std::string version =
    std::to_string(dat1) + "." + std::to_string(dat2) + "."
    + std::to_string(dat3) + "." + std::to_string(dat4) + "."
    + std::to_string(dat5);
  std::cout << "Version of driver is [" << version << "]" << std::endl;

  return version;
}

//////////////////////////////////////////////////
float SEED485Controller::get_voltage()
{
  std::vector<uint8_t> data(6);
  data[0] = 0xFD;
  data[1] = 0xDF;
  data[2] = 0x02;
  data[3] = CMD_GET_TMP_VOLT;
  data[4] = 31;

  // check sum
  int32_t b_check_sum = 0;

  for (size_t i = 2; i < data.size() - 1; ++i) {
    b_check_sum += data[i];
  }

  data[data.size() - 1] =
    ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);

  send_data(data);

  std::vector<uint8_t> dat;
  dat.resize(8);
  read(dat, 8);
  uint16_t header = decode_short_(&dat[0]);
  int16_t cmd;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&cmd);
  bvalue[0] = dat[3];
  bvalue[1] = 0x00;

  if (header != 0xdffd || cmd != CMD_GET_TMP_VOLT) {
    flush();
    //std::cerr << "Proto: ERROR: invalid header" << std::endl;
    return 0;
  }

  float voltage = static_cast<uint16_t>((dat[RAW_HEADER_OFFSET] << 8) + dat[RAW_HEADER_OFFSET + 1]) * 0.1;

  //std::cout << "Voltage is " << voltage << " [V]" << std::endl;

  return voltage;
}

//////////////////////////////////////////////////
void SEED485Controller::read(std::vector<uint8_t>& _read_data, const size_t _length)
{
  _read_data.resize(_length);

  if (ser_.is_open()) {
    boost::mutex::scoped_lock lock(mtx_);

    // Sync to 0xDF 0xFD header (reply header)
    uint8_t sync[2] = {0, 0};
    int retries = 0;
    while (retries < kMaxSyncRetries) {
      std::vector<uint8_t> b(1);
      int size_read = read_some_timed(b, 1, kReadTimeoutMs);
      if (size_read == 1) {
        sync[0] = sync[1];
        sync[1] = b[0];
        if (sync[0] == 0xDF && sync[1] == 0xFD) {
          break;
        }
      }
      // size_read == 0 means the read timed out with no data.
      retries++;
    }

    if (retries >= kMaxSyncRetries) {
      static std::chrono::steady_clock::time_point last_log;
      log_throttled("Proto: ERROR: Could not sync to header", last_log);
      // mtx_ is already held here; flush() would deadlock on it, so
      // flush the port directly instead. Anything still arriving from
      // an abandoned exchange must not be left to corrupt the sync
      // search on the next attempt.
#if ((BOOST_VERSION / 100 % 1000) > 50)
      ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
#else  // 12.04
      ::tcflush(ser_.lowest_layer().native(), TCIOFLUSH);
#endif
      return;
    }

    _read_data[0] = 0xDF;
    _read_data[1] = 0xFD;

    int size = 2;
    int attempts = 0;
    while (size < _length && attempts < kMaxBodyReadAttempts) {
      int size_read;
      std::vector<uint8_t> read_buffer(_length - size);
      size_read = read_some_timed(read_buffer, _length - size, kReadTimeoutMs);
      if (size_read == 0) {
        attempts++;
        continue;
      }
      if ((size + size_read) <= _length) {
        std::copy(read_buffer.begin(), read_buffer.begin()+size_read,
                  _read_data.begin() + size);
      } else {
        std::cerr << "Proto: ERROR: data length, size : " << size << ", size_read " << size_read << std::endl;
        //flush();
#if ((BOOST_VERSION / 100 % 1000) > 50)
        ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
#else  // 12.04
        ::tcflush(ser_.lowest_layer().native(), TCIOFLUSH);
#endif
        break;
      }
      size += size_read;
    }

    if (attempts >= kMaxBodyReadAttempts) {
      static std::chrono::steady_clock::time_point last_log;
      log_throttled("Proto: ERROR: timed out waiting for response body", last_log);
      // Ditto: discard whatever's left of this response so it can't
      // bleed into the next command's header sync.
#if ((BOOST_VERSION / 100 % 1000) > 50)
      ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
#else  // 12.04
      ::tcflush(ser_.lowest_layer().native(), TCIOFLUSH);
#endif
    }
  }

  if (verbose_) {
    std::cout << "recv: ";
    for (size_t i = 0; i < _read_data.size(); ++i) {
      std::cout << std::setw(2)
                << std::uppercase << std::hex
                << std::setw(2) << std::setfill('0')
                << static_cast<int32_t>(_read_data[i]);
    }
    std::cout << "\n";
  }
}

//////////////////////////////////////////////////
void SEED485Controller::send_command(
    uint8_t _cmd, uint16_t _time, std::vector<uint8_t>& _send_data)
{
  send_command(_cmd, 0x00, _time, _send_data);
}

//////////////////////////////////////////////////
void SEED485Controller::send_command(
    uint8_t _cmd, uint8_t _num, uint16_t _data)
{
  std::vector<uint8_t> data(8);
  data[0] = 0xFD;
  data[1] = 0xDF;
  data[2] = 0x04;
  data[3] = _cmd;
  data[4] = _num;
  data[5] = static_cast<uint8_t>(0xff & (_data >> 8));
  data[6] = static_cast<uint8_t>(0xff & _data);

  // check sum
  int32_t b_check_sum = 0;

  for (size_t i = 2; i < data.size() - 1; ++i) {
    b_check_sum += data[i];
  }

  data[data.size() - 1] =
    ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);

  send_data(data);
}

//////////////////////////////////////////////////
void SEED485Controller::send_command(
    uint8_t _cmd, uint8_t _sub, uint16_t _time, std::vector<uint8_t>& _send_data)
{
  _send_data[0] = 0xFD;
  _send_data[1] = 0xDF;
  _send_data[2] = 0x40;
  _send_data[3] = _cmd;
  _send_data[4] = _sub;

  //  time (2 bytes)
  _send_data[65] = static_cast<uint8_t>(0xff & (_time >> 8));
  _send_data[66] = static_cast<uint8_t>(0xff & _time);

  // check sum
  int32_t b_check_sum = 0;

  for (size_t i = 2; i < RAW_DATA_LENGTH - 1; ++i) {
    b_check_sum += _send_data[i];
  }

  _send_data[RAW_DATA_LENGTH - 1] =
    ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);

  send_data(_send_data);
}

//////////////////////////////////////////////////
void SEED485Controller::AERO_Snd_Script(uint16_t sendnum,uint8_t scriptnum)
{
  if (verbose_) {
    struct timeval m_t;
    gettimeofday(&m_t, NULL);
    std::cerr << "[" << m_t.tv_sec << "." << m_t.tv_usec*1000
              <<"] / hand script: " << sendnum
              << ", " << (int)scriptnum << std::endl;
  }
  std::vector<uint8_t> dat(8);
  dat[0] = 0xfd;  // header
  dat[1] = 0xdf;  // header
  dat[2] = 0x04;  // data length
  dat[3] = 0x22;  // execute script
  dat[4] = sendnum;  // sendnum
  dat[5] = 0x00;  //
  dat[6] = scriptnum;  // script No.
  dat[7] = 0xbc;  // checksum
  send_data(dat);
}

//////////////////////////////////////////////////
void SEED485Controller::flush()
{
  if (ser_.is_open()) {
    boost::mutex::scoped_lock lock(mtx_);
#if ((BOOST_VERSION / 100 % 1000) > 50)
    ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
#else  // 12.04
    ::tcflush(ser_.lowest_layer().native(), TCIOFLUSH);
#endif
  }
}

//////////////////////////////////////////////////
void SEED485Controller::send_data(std::vector<uint8_t>& _send_data)
{
  if (ser_.is_open()) {
    boost::mutex::scoped_lock lock(mtx_);
    ser_.write_some(buffer(_send_data));
  }

  if (verbose_) {
    std::cout << "send: ";
    for (size_t i = 0; i < _send_data.size(); ++i) {
      std::cout << std::uppercase << std::hex
		<< std::setw(2) << std::setfill('0')
		<< static_cast<int32_t>(_send_data[i]);
    }
    std::cout << "\n";
  }
}


//////////////////////////////////////////////////
AeroControllerProto::AeroControllerProto(const std::string& _port,
					 uint8_t _id) :
  seed_(_port, _id), verbose_(false), comm_was_lost_(true),
  comm_recovered_latch_(false), comm_fail_streak_(0), comm_ok_streak_(0),
  bad_status_(false)
{
}

//////////////////////////////////////////////////
AeroControllerProto::~AeroControllerProto()
{
}

//////////////////////////////////////////////////
std::string AeroControllerProto::get_version()
{
  return seed_.get_version();
}

//////////////////////////////////////////////////
float AeroControllerProto::get_voltage()
{
  return seed_.get_voltage();
}

//////////////////////////////////////////////////
void AeroControllerProto::servo_on()
{
  servo_command(1);
}

//////////////////////////////////////////////////
void AeroControllerProto::servo_off()
{
  servo_command(0);
}

//////////////////////////////////////////////////
void AeroControllerProto::servo_command(int16_t _d0)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<int16_t> stroke_vector(stroke_joint_indices_.size(), _d0);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat);

  seed_.send_command(CMD_MOTOR_SRV, 0, dat);
}

//////////////////////////////////////////////////
std::vector<int16_t> AeroControllerProto::get_reference_stroke_vector()
{
  return stroke_ref_vector_;
}

//////////////////////////////////////////////////
std::vector<int16_t> AeroControllerProto::get_actual_stroke_vector()
{
  return stroke_cur_vector_;
}

//////////////////////////////////////////////////
std::string AeroControllerProto::get_stroke_joint_name(size_t _idx)
{
  return stroke_joint_indices_[_idx].joint_name;
}

//////////////////////////////////////////////////
int AeroControllerProto::get_number_of_angle_joints()
{
  return angle_joint_indices_.size();
}

//////////////////////////////////////////////////
int AeroControllerProto::get_number_of_strokes()
{
  return stroke_joint_indices_.size();
}

//////////////////////////////////////////////////
int32_t AeroControllerProto::get_ordered_angle_id(std::string _name)
{
  auto it = angle_joint_indices_.find(_name);
  if (it != angle_joint_indices_.end()) {
    return angle_joint_indices_[_name];
  }
  return -1;
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_joint_name(int32_t _joint_id, std::string &_name)
{
  for (auto it = angle_joint_indices_.begin();
       it != angle_joint_indices_.end(); it++ ) {
    if (it->second == _joint_id) {
      _name = it->first;
      return true;
    }
  }
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_status()
{
  return bad_status_;
}

//////////////////////////////////////////////////
std::vector<int16_t> AeroControllerProto::get_status_vec()
{
  return status_vector_;
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_status(std::vector<bool>& _status_vector)
{
  // TODO: status_vector_(int16_t) -> _status_vector(bool)
  return bad_status_;
}

//////////////////////////////////////////////////
void AeroControllerProto::update_position()
{
  if (seed_.is_debug_mode()) {
    // just return ref_vector in debug mode
    stroke_cur_vector_.assign(stroke_ref_vector_.begin(),
                              stroke_ref_vector_.end());
  } else {
    //seed_.flush();
    note_comm_result(get_command(CMD_GET_POS, stroke_cur_vector_));
  }
}

//////////////////////////////////////////////////
void AeroControllerProto::note_comm_result(bool _ok)
{
  if (!_ok) {
    comm_ok_streak_ = 0;
    // Debounced: only a run of kLossStreakThreshold consecutive
    // failures counts as a real loss. A single failed read-back is a
    // routine, self-recovering glitch (e.g. one misaligned byte from
    // USB-serial jitter) that resolves on its own within a cycle or
    // two, and must not be treated as the motor driver losing power.
    if (comm_fail_streak_ < kLossStreakThreshold) {
      comm_fail_streak_++;
    }
    if (comm_fail_streak_ >= kLossStreakThreshold) {
      comm_was_lost_ = true;
    }
    return;
  }

  comm_fail_streak_ = 0;
  if (comm_was_lost_) {
    // Debounced the same way: only report recovery after
    // kRecoveryStreakThreshold consecutive good read-backs, not on the
    // first lucky one, so a full controller restart (stop/start +
    // settle wait) is triggered only for an actual communication
    // recovery (e.g. the servo-on button was cycled while the motor
    // driver had no power), not for a single stray success in the
    // middle of noisy comms.
    comm_ok_streak_++;
    if (comm_ok_streak_ >= kRecoveryStreakThreshold) {
      comm_was_lost_ = false;
      comm_ok_streak_ = 0;
      // The caller (AeroRobotHW) restarts its controllers on this
      // signal so they re-init their setpoints from the measured
      // position instead of snapping back to whatever stale target
      // was set before the loss.
      comm_recovered_latch_ = true;
      std::cerr << "Proto: communication recovered" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
bool AeroControllerProto::check_comm_recovered()
{
  bool recovered = comm_recovered_latch_;
  comm_recovered_latch_ = false;
  return recovered;
}

//////////////////////////////////////////////////
void AeroControllerProto::update_status()
{
  seed_.flush();
  get_command(CMD_WATCH_MISSTEP, status_vector_);
}

//////////////////////////////////////////////////
void AeroControllerProto::reset_status()
{
  seed_.flush();
  get_command(CMD_WATCH_MISSTEP, 0xff, status_vector_);
}

//////////////////////////////////////////////////
void AeroControllerProto::get_current(std::vector<int16_t>& _stroke_vector)
{
  get_command(CMD_GET_CUR, _stroke_vector);
}

//////////////////////////////////////////////////
void AeroControllerProto::get_temperature(
    std::vector<int16_t>& _stroke_vector)
{
  get_command(CMD_GET_TMP_VOLT, _stroke_vector);
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_data(std::vector<int16_t>& _stroke_vector)
{
  std::vector<uint8_t> dat;
  dat.resize(RAW_DATA_LENGTH);

  seed_.read(dat);

  uint16_t header = decode_short_(&dat[0]);
  int16_t cmd;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&cmd);
  bvalue[0] = dat[3];
  bvalue[1] = 0x00;

  if (header != 0xdffd) {
    seed_.flush();
    static std::chrono::steady_clock::time_point last_log;
    log_throttled("Proto: ERROR: invalid header", last_log);
    return false;
  }

  bool matched = false;
  if (cmd == CMD_MOVE_ABS_POS ||
      cmd == CMD_MOVE_ABS_POS_RET ||
      cmd == CMD_GET_POS ||
      cmd == CMD_GET_CUR ||
      cmd == CMD_GET_TMP_VOLT ||
      cmd == CMD_GET_AD ||
      cmd == CMD_GET_DIO ||
      cmd == CMD_WATCH_MISSTEP) {
    matched = true;
    _stroke_vector.resize(stroke_joint_indices_.size());
    // raw to stroke
    for (size_t i = 0; i < stroke_joint_indices_.size(); ++i) {
      AJointIndex& aji = stroke_joint_indices_[i];
      // uint8_t -> uint16_t
      _stroke_vector[aji.stroke_index] =
        decode_short_(&dat[RAW_HEADER_OFFSET + aji.raw_index * 2]);

      // check value
      if (_stroke_vector[aji.stroke_index] > 0x7fff) {
        _stroke_vector[aji.stroke_index] -= std::pow(2, 16);
      } else if (_stroke_vector[aji.stroke_index] == 0x7fff) {
        _stroke_vector[aji.stroke_index] = 0;
      }
    }
  }

  // if (cmd == CMD_MOVE_ABS || cmd == CMD_WATCH_MISSTEP || cmd == CMD_GET_POS) {
  if (cmd == CMD_WATCH_MISSTEP) {
    uint8_t status0 = dat[RAW_HEADER_OFFSET + 60];
    uint8_t status1 = dat[RAW_HEADER_OFFSET + 61];
    if ((status0 >> 5) == 1 || (status1 >> 5) == 1) {
      bad_status_ = true;
    } else {
      bad_status_ = false;
    }
  }

  return matched;
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_command(uint8_t _cmd,
                                      std::vector<int16_t>& _stroke_vector)
{
  return get_command(_cmd, 0x00, _stroke_vector);
}

//////////////////////////////////////////////////
bool AeroControllerProto::get_command(uint8_t _cmd, uint8_t _sub,
                                      std::vector<int16_t>& _stroke_vector)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  // The original code sent the command exactly once and then waited
  // (unboundedly) for a reply. If the motor driver wasn't listening
  // at that exact moment -- e.g. mid power-on calibration, right after
  // the servo-on button is pressed -- the command is simply lost on
  // the wire, and there was nothing to ever resend it: the code just
  // waited forever for a reply to a request the hardware never got.
  // Resending here a few times closes that gap.
  for (int attempt = 0; attempt < kMaxCommandRetries; ++attempt) {
    std::vector<uint8_t> dat(RAW_DATA_LENGTH);
    seed_.send_command(_cmd, _sub, 0, dat);
    if (get_data(_stroke_vector)) {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
void AeroControllerProto::set_position(
    std::vector<int16_t>& _stroke_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  // for ROS
  for (size_t i = 0; i < _stroke_vector.size(); ++i) {
    if (_stroke_vector[i] != 0x7fff) {
      stroke_ref_vector_[i] = _stroke_vector[i];
    }
  }

  // for seed
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);
  stroke_to_raw_(_stroke_vector, dat);
  //seed_.flush();
  seed_.send_command(CMD_MOVE_ABS_POS_RET, _time, dat);

  // for ROS
  if (seed_.is_debug_mode()) {
    usleep(1000 * 20);
    // in debug mode, get_data returns before writing stroke vector,
    // and controller must copy ref_vector into cur_vector
    stroke_cur_vector_.assign(stroke_ref_vector_.begin(),
                              stroke_ref_vector_.end());
  } else {
    // this, not update_position(), is the read-back that runs every
    // control cycle, so communication loss/recovery is tracked here
    note_comm_result(get_data(stroke_cur_vector_));
  }
}

//////////////////////////////////////////////////
void AeroControllerProto::set_position_no_wait(
    std::vector<int16_t>& _stroke_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  // for ROS
  for (size_t i = 0; i < _stroke_vector.size(); ++i) {
    if (_stroke_vector[i] != 0x7fff) {
      stroke_ref_vector_[i] = _stroke_vector[i];
    }
  }

  // for seed
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);
  stroke_to_raw_(_stroke_vector, dat);
  //seed_.flush();
  seed_.send_command(CMD_MOVE_ABS_POS, _time, dat);

  // for ROS
  if (seed_.is_debug_mode()) {
    // in debug mode, get_data returns before writing stroke vector,
    // and controller must copy ref_vector into cur_vector
    stroke_cur_vector_.assign(stroke_ref_vector_.begin(),
                              stroke_ref_vector_.end());
  } else {
    // no return
    //get_data(stroke_cur_vector_);
  }
}

//////////////////////////////////////////////////
void AeroControllerProto::set_max_single_current(int8_t _num, int16_t _dat)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);
  if (verbose_) {
    struct timeval m_t;
    gettimeofday(&m_t, NULL);
    std::cerr << "[" << m_t.tv_sec << "." << m_t.tv_usec*1000
              << "] / max cur: " << (int)_num
              << ", " << _dat << std::endl;
  }
  seed_.send_command(CMD_MOTOR_CUR, _num, _dat);
}

//////////////////////////////////////////////////
void AeroControllerProto::set_max_current(
    std::vector<int16_t>& _stroke_vector)
{
  set_command(CMD_MOTOR_CUR, _stroke_vector);
}

//////////////////////////////////////////////////
void AeroControllerProto::set_accel_rate(
    std::vector<int16_t>& _stroke_vector)
{
  set_command(CMD_MOTOR_ACC, _stroke_vector);
}

//////////////////////////////////////////////////
void AeroControllerProto::set_motor_gain(
    std::vector<int16_t>& _stroke_vector)
{
  set_command(CMD_MOTOR_GAIN, _stroke_vector);
}

//////////////////////////////////////////////////
void AeroControllerProto::set_command(
    uint8_t _cmd, uint8_t _num, uint16_t _data)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);
  seed_.send_command(_cmd, _num, _data);
}

//////////////////////////////////////////////////
void AeroControllerProto::set_command(uint8_t _cmd,
                                      std::vector<int16_t>& _stroke_vector)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);
  stroke_to_raw_(_stroke_vector, dat);
  seed_.send_command(_cmd, 0, dat);
}

//////////////////////////////////////////////////
void AeroControllerProto::stroke_to_raw_(std::vector<int16_t>& _stroke,
                                         std::vector<uint8_t>& _raw)
{
  for (size_t i = 0; i < stroke_joint_indices_.size(); ++i) {
    AJointIndex& aji = stroke_joint_indices_[i];
    // uint16_t -> uint8_t
    encode_short_(_stroke[aji.stroke_index],
                  &_raw[RAW_HEADER_OFFSET + aji.raw_index * 2]);
  }
}

//////////////////////////////////////////////////
int16_t aero::controller::decode_short_(uint8_t* _raw)
{
  int16_t value;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  bvalue[0] = _raw[1];
  bvalue[1] = _raw[0];
  return value;
}

//////////////////////////////////////////////////
void aero::controller::encode_short_(int16_t _value, uint8_t* _raw)
{
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&_value);
  _raw[0] = bvalue[1];
  _raw[1] = bvalue[0];
}
