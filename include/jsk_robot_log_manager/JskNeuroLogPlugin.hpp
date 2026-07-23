#ifndef JSK_ROBOT_LOG_MANAGER_JSK_NEURO_LOG_PLUGIN_HPP_
#define JSK_ROBOT_LOG_MANAGER_JSK_NEURO_LOG_PLUGIN_HPP_

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include "AbstractPlugin.hpp"
#include "RobotState.hpp"

class JskNeuroLogPlugin final : public AbstractPlugin
{
public:
  explicit JskNeuroLogPlugin(
    std::initializer_list<unsigned short> candidate_ports = {7001, 7002},
    const std::string& listen_host = "127.0.0.1",
    unsigned int log_period_usec = 1000) :
    AbstractPlugin(AbstractPlugin::TYPE::ASYNC, log_period_usec),
    candidate_ports(candidate_ports),
    listen_host(listen_host)
  {
    this->name = std::to_string(this->id) + ":JskNeuroLogPlugin";
    this->addDefaultStreams();
  }

  ~JskNeuroLogPlugin()
  {
    this->abort();
    this->stopRecording();
    this->server_alive = false;
    if (this->server_fd >= 0)
      {
        shutdown(this->server_fd, SHUT_RDWR);
        close(this->server_fd);
        this->server_fd = -1;
      }
    int connection_fd = this->active_connection_fd.load();
    if (connection_fd >= 0)
      {
        shutdown(connection_fd, SHUT_RDWR);
      }
    if (this->server_thread.joinable())
      {
        this->server_thread.join();
      }
  }

  void initialize(void) override
  {
    this->routine_active = true;
    if (!this->openServer())
      {
        this->messenger
          << this->messageHeader()
          << "could not bind any configured command port"
          << std::endl;
        return;
      }

    this->server_alive = true;
    this->server_thread = std::thread(
      &JskNeuroLogPlugin::serveCommands,
      this);
    this->messenger
      << this->messageHeader()
      << "command server listening on "
      << this->listen_host
      << ":"
      << this->selected_port
      << std::endl;
  }

  void execute(void) override
  {
    std::lock_guard<std::mutex> lock(this->log_mutex);
    if (!this->recording)
      {
        return;
      }

    RobotState& robot = *RobotState::instance();
    double timestamp =
      static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - this->start_time).count()
      ) / 1000000.0;

    for (StreamConfig& config : this->stream_list)
      {
        config.stream << this->sample_index << " " << timestamp;
        this->writeStream(config, robot);
        config.stream << std::endl;
      }
    ++this->sample_index;
  }

  bool executeCommand(
    const std::list<std::string>& command,
    std::string& response) override
  {
    if (command.empty() || command.front() != "jsklog")
      {
        return false;
      }
    if (command.size() < 2)
      {
        response = "ERROR usage: jsklog start <absolute-prefix>|stop\n";
        return true;
      }

    const std::string& action = *std::next(command.begin(), 1);
    if (action == "stop")
      {
        response = this->stopRecording();
        return true;
      }
    if (action == "start" && command.size() >= 3)
      {
        response = this->startRecording(*std::next(command.begin(), 2));
        return true;
      }

    response = "ERROR usage: jsklog start <absolute-prefix>|stop\n";
    return true;
  }

  unsigned short port(void) const
  {
    return this->selected_port;
  }

private:
  enum class StreamType
  {
    JOINT_Q,
    JOINT_DQ,
    JOINT_TAU,
    JOINT_Q_REF,
    JOINT_DQ_REF,
    JOINT_TAU_REF,
    BODY_FORCE,
    DEVICE_RAW,
    DEVICE_CONVERTED,
    TIMER,
    STOPWATCH,
  };

  struct StreamConfig
  {
    StreamType type;
    std::size_t data_id;
    RobotDeviceState::SLOT slot;
    std::string name;
    std::ofstream stream;

    StreamConfig(
      StreamType type,
      const std::string& name,
      std::size_t data_id = 0,
      RobotDeviceState::SLOT slot = RobotDeviceState::SLOT::END) :
      type(type),
      data_id(data_id),
      slot(slot),
      name(name)
    {
    }
  };

  std::vector<unsigned short> candidate_ports;
  std::string listen_host;
  unsigned short selected_port{0};
  int server_fd{-1};
  std::atomic<int> active_connection_fd{-1};
  std::atomic<bool> server_alive{false};
  std::thread server_thread;

  std::mutex log_mutex;
  bool recording{false};
  std::string output_prefix;
  unsigned int sample_index{0};
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  std::list<StreamConfig> stream_list;
  std::list<RobotDeviceState*> device_list;

  void addDefaultStreams(void)
  {
    this->stream_list.emplace_back(StreamType::JOINT_Q, "q");
    this->stream_list.emplace_back(StreamType::JOINT_DQ, "dq");
    this->stream_list.emplace_back(StreamType::JOINT_TAU, "tau");
    this->stream_list.emplace_back(StreamType::JOINT_Q_REF, "q_ref");
    this->stream_list.emplace_back(StreamType::JOINT_DQ_REF, "dq_ref");
    this->stream_list.emplace_back(StreamType::JOINT_TAU_REF, "tau_ref");
    this->stream_list.emplace_back(StreamType::BODY_FORCE, "force0", 0);
    this->stream_list.emplace_back(StreamType::BODY_FORCE, "force1", 1);
    this->stream_list.emplace_back(StreamType::TIMER, "timer");
    this->stream_list.emplace_back(StreamType::STOPWATCH, "stopwatch");

    this->addRawDeviceStream(
      RobotDeviceState::SLOT::CONTROL_STATUS,
      "status");
    this->addRawDeviceStream(
      RobotDeviceState::SLOT::ENCODER,
      "enc");
    this->addRawDeviceStream(
      RobotDeviceState::SLOT::ENCODER_REFERENCE_READBACK,
      "enc_ref");
    this->addRawDeviceStream(
      RobotDeviceState::SLOT::ENCODER_VELOCITY,
      "vel");
    this->addRawDeviceStream(
      RobotDeviceState::SLOT::ENCODER_VELOCITY_REFERENCE_READBACK,
      "vel_ref");
    this->addRawDeviceStream(
      RobotDeviceState::SLOT::JOINT_ENCODER,
      "abs_enc");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_TEMPERATURE,
      "temp");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_ID,
      "id");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_IQ,
      "iq");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_IQ_REFERENCE,
      "iq_out");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_IU,
      "iu");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_IV,
      "iv");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::MOTOR_IW,
      "iw");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::VIN,
      "vin");
    this->addConvertedDeviceStream(
      RobotDeviceState::SLOT::VDD,
      "vdd");
  }

  void addRawDeviceStream(
    RobotDeviceState::SLOT slot,
    const std::string& name)
  {
    this->stream_list.emplace_back(
      StreamType::DEVICE_RAW,
      name,
      0,
      slot);
  }

  void addConvertedDeviceStream(
    RobotDeviceState::SLOT slot,
    const std::string& name)
  {
    this->stream_list.emplace_back(
      StreamType::DEVICE_CONVERTED,
      name,
      0,
      slot);
  }

  bool openServer(void)
  {
    for (unsigned short port : this->candidate_ports)
      {
        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        struct addrinfo* addresses = nullptr;
        std::string port_name = std::to_string(port);
        int result = getaddrinfo(
          this->listen_host.c_str(),
          port_name.c_str(),
          &hints,
          &addresses);
        if (result != 0 || addresses == nullptr)
          {
            if (addresses != nullptr)
              {
                freeaddrinfo(addresses);
              }
            continue;
          }

        int candidate_fd = socket(
          addresses->ai_family,
          addresses->ai_socktype,
          addresses->ai_protocol);
        if (candidate_fd < 0)
          {
            freeaddrinfo(addresses);
            continue;
          }

        int reuse_address = 1;
        setsockopt(
          candidate_fd,
          SOL_SOCKET,
          SO_REUSEADDR,
          &reuse_address,
          sizeof(reuse_address));

        bool bound =
          bind(
            candidate_fd,
            addresses->ai_addr,
            addresses->ai_addrlen) == 0;
        freeaddrinfo(addresses);
        if (!bound || listen(candidate_fd, 5) != 0)
          {
            close(candidate_fd);
            continue;
          }

        this->server_fd = candidate_fd;
        this->selected_port = port;
        return true;
      }
    return false;
  }

  void serveCommands(void)
  {
    while (this->server_alive)
      {
        int connection_fd = accept(this->server_fd, nullptr, nullptr);
        if (connection_fd < 0)
          {
            if (this->server_alive)
              {
                this->messenger
                  << this->messageHeader()
                  << "accept failed: "
                  << std::strerror(errno)
                  << std::endl;
              }
            break;
          }
        this->active_connection_fd = connection_fd;

        std::string command;
        char buffer[4096];
        while (command.find('\n') == std::string::npos &&
               command.size() <= 65536)
          {
            ssize_t received = recv(
              connection_fd,
              buffer,
              sizeof(buffer),
              0);
            if (received <= 0)
              {
                break;
              }
            command.append(buffer, static_cast<std::size_t>(received));
          }

        std::size_t newline = command.find('\n');
        if (newline != std::string::npos)
          {
            command.erase(newline);
          }
        if (!command.empty())
          {
            std::string response = this->handleSocketCommand(command);
            this->sendAll(connection_fd, response);
          }
        this->active_connection_fd = -1;
        close(connection_fd);
      }
  }

  std::string handleSocketCommand(const std::string& command)
  {
    const std::string start_command = "log start ";
    if (command.compare(0, start_command.size(), start_command) == 0)
      {
        return this->startRecording(command.substr(start_command.size()));
      }
    if (command == "log stop")
      {
        return this->stopRecording();
      }
    if (command == "log status")
      {
        std::lock_guard<std::mutex> lock(this->log_mutex);
        if (this->recording)
          {
            return "OK log recording " + this->output_prefix + "\n";
          }
        return "OK log stopped\n";
      }
    return "ERROR unknown command\n";
  }

  std::string startRecording(const std::string& prefix)
  {
    std::lock_guard<std::mutex> lock(this->log_mutex);
    if (this->recording)
      {
        return "ERROR log already active\n";
      }
    if (prefix.empty() || prefix.front() != '/')
      {
        return "ERROR log prefix must be an absolute path\n";
      }

    this->refreshDeviceList();
    for (StreamConfig& config : this->stream_list)
      {
        config.stream.clear();
        std::string file_name =
          prefix + "_" + config.name + ".dat";
        config.stream.open(
          file_name,
          std::ios_base::out | std::ios_base::trunc);
        if (!config.stream.is_open())
          {
            this->closeStreams();
            return "ERROR could not open " + file_name + "\n";
          }
      }

    this->output_prefix = prefix;
    this->sample_index = 0;
    this->start_time = std::chrono::steady_clock::now();
    this->recording = true;
    return "OK log started " + prefix + "\n";
  }

  std::string stopRecording(void)
  {
    std::lock_guard<std::mutex> lock(this->log_mutex);
    if (!this->recording)
      {
        return "ERROR log already stopped\n";
      }
    this->recording = false;
    this->closeStreams();
    return "OK log stopped\n";
  }

  void closeStreams(void)
  {
    for (StreamConfig& config : this->stream_list)
      {
        if (config.stream.is_open())
          {
            config.stream.close();
          }
      }
  }

  void refreshDeviceList(void)
  {
    this->device_list.clear();
    for (std::unique_ptr<RobotDeviceState>& state :
         RobotState::instance()->device)
      {
        this->device_list.push_back(state.get());
        for (std::unique_ptr<RobotDeviceState>& child : state->child)
          {
            this->device_list.push_back(child.get());
          }
      }
  }

  void writeStream(StreamConfig& config, RobotState& robot)
  {
    switch (config.type)
      {
      case StreamType::JOINT_Q:
        this->writeValues(config.stream, robot.joint_q);
        break;
      case StreamType::JOINT_DQ:
        this->writeValues(config.stream, robot.joint_dq);
        break;
      case StreamType::JOINT_TAU:
        this->writeValues(config.stream, robot.joint_tau);
        break;
      case StreamType::JOINT_Q_REF:
        this->writeValues(config.stream, robot.joint_q_ref);
        break;
      case StreamType::JOINT_DQ_REF:
        this->writeValues(config.stream, robot.joint_dq_ref);
        break;
      case StreamType::JOINT_TAU_REF:
        this->writeValues(config.stream, robot.joint_tau_ref);
        break;
      case StreamType::BODY_FORCE:
        if (robot.endeffector_force.size() > config.data_id)
          {
            this->writeValues(
              config.stream,
              robot.endeffector_force.at(config.data_id));
          }
        break;
      case StreamType::DEVICE_RAW:
        for (RobotDeviceState* device : this->device_list)
          {
            config.stream << " " << device->getRaw(config.slot);
          }
        break;
      case StreamType::DEVICE_CONVERTED:
        for (RobotDeviceState* device : this->device_list)
          {
            config.stream << " " << device->getConverted(config.slot);
          }
        break;
      case StreamType::TIMER:
        config.stream << " " << robot.timer.period();
        config.stream << " " << robot.timer.loopCounter();
        config.stream << " " << robot.timer.lastRuntime();
        config.stream << " " << robot.timer.lastInterval();
        config.stream << " " << robot.timer.runtimeAverage();
        config.stream << " " << robot.timer.intervalAverage();
        config.stream << " " << robot.timer.intervalJitter();
        break;
      case StreamType::STOPWATCH:
        {
          std::list<unsigned int> laps;
          unsigned int total = robot.stopwatch.get(laps);
          config.stream << " " << total;
          for (unsigned int lap : laps)
            {
              config.stream << " " << lap;
            }
          break;
        }
      }
  }

  template<class Values>
  void writeValues(std::ofstream& stream, const Values& values)
  {
    for (const auto& value : values)
      {
        stream << " " << value;
      }
  }

  void sendAll(int connection_fd, const std::string& response)
  {
    std::size_t sent_size = 0;
    while (sent_size < response.size())
      {
        ssize_t sent = send(
          connection_fd,
          response.data() + sent_size,
          response.size() - sent_size,
          MSG_NOSIGNAL);
        if (sent <= 0)
          {
            return;
          }
        sent_size += static_cast<std::size_t>(sent);
      }
  }
};

#endif
