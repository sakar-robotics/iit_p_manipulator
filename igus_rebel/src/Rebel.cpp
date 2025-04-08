#include "rclcpp/rclcpp.hpp"
#include "igus_rebel/Rebel.hpp"
#include "igus_rebel/CriKeywords.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <iostream>
#include <sstream>

namespace Igus
{

    //
    // Constructor(s) / Destructor(s)
    //
    Rebel::Rebel()
    {
    }

    Rebel::~Rebel()
    {
        Stop();
    }

    //
    // private functions
    //
    void Rebel::AliveThreadFunction()
    {
        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Starting to send ALIVEJOG");

        while (continueAlive)
        {
            std::ostringstream msg;
            msg << std::showpoint;
            msg << std::fixed;
            msg << std::setprecision(8);
            msg << "CRISTART " << Ccnt() << " ";
            msg << "ALIVEJOG ";
            msg << j1 << " " << j2 << " " << j3 << " ";
            msg << j4 << " " << j5 << " " << j6 << " ";
            msg << 0.0f << " " << 0.0f << " " << 0.0f << " ";
            msg << "CRIEND" << std::endl;

            // std::cout << msg.str();
            {
                std::lock_guard<std::mutex> lockGuard(aliveLock);
                rebelSocket->SendMessage(msg.str());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs));
        }

        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Stopped to send ALIVEJOG");
    }

    void Rebel::MessageThreadFunction()
    {
        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Starting to process robot messages");

        while (continueMessage)
        {
            if (rebelSocket->HasMessage())
            {
                std::string msg = rebelSocket->GetMessage();

                CriMessages::MessageType type = CriMessages::CriMessage::GetMessageType(msg);

                switch (type)
                {
                case CriMessages::MessageType::STATUS:
                {
                    CriMessages::Status status = CriMessages::Status(msg);
                    // status.Print();
                    status.Log();
                    currentStatus = status;
                    ProcessStatus(currentStatus);
                    break;
                }

                case CriMessages::MessageType::RUNSTATE:
                {
                    break;
                }

                case CriMessages::MessageType::MESSAGE:
                {
                    CriMessages::Message message = CriMessages::Message(msg);
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel MESSAGE: %s", message.message.c_str());
                    break;
                }

                case CriMessages::MessageType::CMD:
                {
                    CriMessages::Command command = CriMessages::Command(msg);

                    // Not sure if the ROS node should display these?
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "CMD: %s", command.command.c_str());
                    break;
                }

                case CriMessages::MessageType::CONFIG:
                {
                    CriMessages::ConfigType configType = CriMessages::Config::GetConfigType(msg);

                    switch (configType)
                    {
                    case CriMessages::ConfigType::KINEMATICLIMITS:
                    {
                        CriMessages::KinematicLimits kinematicLimits = CriMessages::KinematicLimits(msg);
                        kinematicLimits.Print();
                        break;
                    }
                    case CriMessages::ConfigType::UNKNOWN:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Unknown config message: %s", msg.c_str());
                        break;
                    }
                    }

                    break;
                }

                case CriMessages::MessageType::INFO:
                {
                    CriMessages::Info info = CriMessages::Info(msg);
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "INFO: %s", info.info.c_str());
                    break;
                }

                case CriMessages::MessageType::LOGMSG:
                {
                    CriMessages::LogMsg log = CriMessages::LogMsg(msg);

                    switch (log.logLevel)
                    {
                    case CriMessages::LogLevel::DEBUG:
                    {
                        RCLCPP_DEBUG(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::APP_INFO:
                    {
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "REBEL LOG (APP_INFO): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::APP_ERROR:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG (APP_ERROR): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::INFO:
                    {
                        // The Rebel is pretty chatty with its INFO level log messages, so I've set them to output only to the ROS DEBUG level.
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::WARN:
                    {
                        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::ERROR:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::FATAL:
                    {
                        RCLCPP_FATAL(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::UNKNOWN:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG (UNKNOWN LOG LEVEL): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }
                    }

                    break;
                }

                case CriMessages::MessageType::VARIABLES:
                {
                    // CriMessages::Variables vars = CriMessages::Variables(msg);
                    break;
                }

                case CriMessages::MessageType::CMDERROR:
                {
                    CriMessages::CmdError error = CriMessages::CmdError(msg);

                    try
                    {
                        std::string command = unacknowledgedCommands.at(error.recjectedCmd);
                        unacknowledgedCommands.erase(error.recjectedCmd);
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Rebel did not accept command: %s. Error message: %s", command.c_str(), error.error.c_str());
                    }
                    catch (const std::out_of_range &e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Rebel did not accept unknown command. Error message: %s (%d)", error.error.c_str(), error.recjectedCmd);
                    }
                    break;
                }

                case CriMessages::MessageType::CMDACK:
                {
                    CriMessages::CmdAck ack = CriMessages::CmdAck(msg);

                    try
                    {
                        std::string command = unacknowledgedCommands.at(ack.acceptedCmd);
                        unacknowledgedCommands.erase(ack.acceptedCmd);
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel accepted command: %s", command.c_str());
                        break;
                    }
                    catch (const std::out_of_range &e)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Rebel accepted unknown command: %d", ack.acceptedCmd);
                        break;
                    }
                    break;
                }

                case CriMessages::MessageType::CYCLESTAT:
                {
                    CriMessages::Cyclestat cyclestat = CriMessages::Cyclestat(msg);
                    // Will only output this once every 2 minutes, because this is sent every 0.5 seconds.
                    // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("igus_rebel"), *node_->get_clock(), 120, "Rebel cycle statistics -- Cycletime: %d -- Workload: %d%%", cyclestat.cycletime, cyclestat.workload);
                    break;
                }

                case CriMessages::MessageType::UNKNOWN:
                {
                    RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "UNKNOW MESSAGE: %s", msg.c_str());
                    break;
                }

                case CriMessages::MessageType::OPINFO:
                {
                    break;
                }

                case CriMessages::MessageType::GSIG:
                {
                    break;
                }
                case CriMessages::MessageType::GRIPPERSTATE:
                {
                    break;
                }
                }
            }
        }

        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Stopped to process robot messages");
    }

    int Rebel::Ccnt()
    {
        std::lock_guard<std::mutex> lockGuard(cntLock);
        int current = current_ccnt;
        current_ccnt = (current_ccnt % 9999) + 1;
        return current;
    }

    void Rebel::SetDigitalOut(const int &output, const bool &is_on)
    {
        std::ostringstream cmd;
        cmd << CriKeywords::COMMAND_DOUT << " " << output << " " << (is_on ? "true" : "false");
        Command(cmd.str());
    }

    void Rebel::Command(const std::string &command)
    {
        int commandCount = Ccnt();
        std::ostringstream msg;
        msg << CriKeywords::START << " " << commandCount << " ";
        msg << CriKeywords::TYPE_CMD << " ";
        msg << command << " ";
        msg << CriKeywords::END << std::endl;

        unacknowledgedCommands[commandCount] = command;

        rebelSocket->SendMessage(msg.str());
    }

    void Rebel::GetConfig(const std::string &config)
    {
        std::ostringstream msg;
        msg << CriKeywords::START << " " << Ccnt() << " ";
        msg << CriKeywords::TYPE_CONFIG << " ";
        msg << config << " ";
        msg << CriKeywords::END << std::endl;

        rebelSocket->SendMessage(msg.str());
    }

    void Rebel::SetControlMode(const ControlMode &mode)
    {
        switch (mode)
        {
        case Rebel::ControlMode::POSITION:
        {
            {
                std::lock_guard<std::mutex> lockGuard(aliveLock);

                j1 = currentStatus.posJointCurrent.at(0);
                j2 = currentStatus.posJointCurrent.at(1);
                j3 = currentStatus.posJointCurrent.at(2);
                j4 = currentStatus.posJointCurrent.at(3);
                j5 = currentStatus.posJointCurrent.at(4);
                j6 = currentStatus.posJointCurrent.at(5);

                Command(CriKeywords::COMMAND_MOTIONTYPECARTBASE);
                controlMode = mode;
            }
            RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel now controlled by position control.");
            break;
        }

        case Rebel::ControlMode::VELOCITY:
        {
            Command(CriKeywords::COMMAND_MOTIONTYPEJOINT);
            controlMode = mode;
            RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel now controlled by velocity control.");
            break;
        }
        }
    }

    void Rebel::ProcessStatus(const CriMessages::Status &status)
    {
        CriMessages::Kinstate currentKinstate = status.kinstate;
        std::array<int, 16> currentErrorJoints = status.errorJoints;

        if (lastKinstate != currentKinstate)
        {

            if (lastKinstate != CriMessages::Kinstate::NO_ERROR)
            {
                RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Kinematics error resolved [%s]", kinstateMessage.c_str());
            }

            if (currentKinstate != CriMessages::Kinstate::NO_ERROR)
            {

                switch (status.kinstate)
                {
                case CriMessages::Kinstate::JOINT_LIMIT_MIN:
                {
                    kinstateMessage = "joint at minimum limit";
                    break;
                }

                case CriMessages::Kinstate::JOINT_LIMIT_MAX:
                {
                    kinstateMessage = "joint at maximum limit";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_CENTER:
                {
                    kinstateMessage = "cartesian singularity (center)";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_REACH:
                {
                    kinstateMessage = "cartesian singularity (reach)";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_WRIST:
                {
                    kinstateMessage = "cartesian singularity (wrist)";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_1:
                {
                    kinstateMessage = "tool at virtual box limit 1";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_2:
                {
                    kinstateMessage = "tool at virtual box limit 2";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_3:
                {
                    kinstateMessage = "tool at virtual box limit 3";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_4:
                {
                    kinstateMessage = "tool at virtual box limit 4";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_5:
                {
                    kinstateMessage = "tool at virtual box limit 5";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_6:
                {
                    kinstateMessage = "tool at virtual box limit 6";
                    break;
                }

                case CriMessages::Kinstate::MOTION_NOT_ALLOWED:
                {
                    kinstateMessage = "motion not allowed";
                    break;
                }

                case CriMessages::Kinstate::UNKNOWN:
                {
                    kinstateMessage = "unknown error";
                    break;
                }

                case CriMessages::Kinstate::NO_ERROR:
                {
                    kinstateMessage = "no error";
                    break;
                }
                }

                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Kinematics error [%s]", kinstateMessage.c_str());
            }
        }

        if (currentErrorJoints != lastErrorJoints)
        {

            // loop throught the 6 joint errors
            for (unsigned int i = 0; i < 6; i++)
            {
                int errorJoint = currentErrorJoints.at(i);
                std::array<int, 8> errorJointBit;

                if (errorJoint != lastErrorJoints.at(i))
                {

                    // extract bits from the error to analyze it
                    for (unsigned j = 0; j < 8; j++)
                    {
                        errorJointBit[j] = errorJoint & (int)exp2(j);
                    }

                    std::string errorMsg = "";
                    if (errorJointBit.at(0) == static_cast<int>(CriMessages::ErrorJoint::TEMP))
                    {
                        errorMsg += "'Overtemperature' ";
                    }

                    if (errorJointBit.at(1) == static_cast<int>(CriMessages::ErrorJoint::ESTOP_LOWV))
                    {
                        errorMsg += "'Supply too low: Is emergency button pressed?' ";
                    }

                    if (errorJointBit.at(2) == static_cast<int>(CriMessages::ErrorJoint::MNE))
                    {
                        errorMsg += "'Motor not enabled' ";
                    }

                    if (errorJointBit.at(3) == static_cast<int>(CriMessages::ErrorJoint::COM))
                    {
                        errorMsg += "'Communication watch dog' ";
                    }

                    if (errorJointBit.at(4) == static_cast<int>(CriMessages::ErrorJoint::POS))
                    {
                        errorMsg += "'Position lag' ";
                    }

                    if (errorJointBit.at(5) == static_cast<int>(CriMessages::ErrorJoint::ENC))
                    {
                        errorMsg += "'Encoder Error' ";
                    }

                    if (errorJointBit.at(6) == static_cast<int>(CriMessages::ErrorJoint::OC))
                    {
                        errorMsg += "'Overcurrent' ";
                    }

                    if (errorJointBit.at(7) == static_cast<int>(CriMessages::ErrorJoint::DRV))
                    {
                        errorMsg += "'DriveError/SVM' ";
                    }

                    if (errorMsg != "")
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Joint %i Error: [%s]", i, errorMsg.c_str());
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Joint %i Error: Cleared", i);
                    }
                }
            }
        }

        lastKinstate = currentKinstate;
        lastErrorJoints = currentErrorJoints;
    }

    //
    // public functions
    //
    void Rebel::SetJog(const float &joint1, const float &joint2, const float &joint3,
                       const float &joint4, const float &joint5, const float &joint6)
    {
        j1 = joint1;
        j2 = joint2;
        j3 = joint3;
        j4 = joint4;
        j5 = joint5;
        j6 = joint6;
    }

    void Rebel::GetJoints(float &joint1, float &joint2, float &joint3,
                          float &joint4, float &joint5, float &joint6)
    {
        joint1 = currentStatus.posJointCurrent.at(0);
        joint2 = currentStatus.posJointCurrent.at(1);
        joint3 = currentStatus.posJointCurrent.at(2);
        joint4 = currentStatus.posJointCurrent.at(3);
        joint5 = currentStatus.posJointCurrent.at(4);
        joint6 = currentStatus.posJointCurrent.at(5);
    }

    CallbackReturn Rebel::on_init(const HardwareInfo &)
    {
        rebelSocket = std::make_shared<RebelSocket>(ip, port, 200),
        j1 = 0.0f;
        j2 = 0.0f;
        j3 = 0.0f;
        j4 = 0.0f;
        j5 = 0.0f;
        j6 = 0.0f;
        controlMode = Rebel::ControlMode::VELOCITY;
        current_ccnt = 1;
        continueAlive = false;
        continueMessage = false;
        aliveWaitMs = 100;
        lastKinstate = CriMessages::Kinstate::NO_ERROR;
        kinstateMessage = "";
        node_ = std::make_shared<rclcpp::Node>("igus_rebel");
        digital_output_srv_ = node_->create_service<igus_rebel_msgs::srv::SetDigitalOutput>(
            "set_digital_output", std::bind(&Rebel::dio_callback, this, std::placeholders::_1, std::placeholders::_2));
        Start();
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> Rebel::export_state_interfaces()
    {
        std::vector<StateInterface> state_interfaces;

        for (int i = 0; i < 6; ++i)
        {
            state_interfaces.emplace_back(StateInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_POSITION, &pos[i]));
            state_interfaces.emplace_back(StateInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_VELOCITY, &vel[i]));
        }

        return state_interfaces;
    }

    std::vector<CommandInterface> Rebel::export_command_interfaces()
    {
        std::vector<CommandInterface> command_interfaces;

        for (int i = 0; i < 6; ++i)
        {
            command_interfaces.emplace_back(CommandInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd[i]));
        }

        return command_interfaces;
    }

    return_type Rebel::read(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        read();

        vel[0] = (pos[0] - last_pos[0]) / period.seconds();
        vel[1] = (pos[1] - last_pos[1]) / period.seconds();
        vel[2] = (pos[2] - last_pos[2]) / period.seconds();
        vel[3] = (pos[3] - last_pos[3]) / period.seconds();
        vel[4] = (pos[4] - last_pos[4]) / period.seconds();
        vel[5] = (pos[5] - last_pos[5]) / period.seconds();

        last_pos[0] = pos[0];
        last_pos[1] = pos[1];
        last_pos[2] = pos[2];
        last_pos[3] = pos[3];
        last_pos[4] = pos[4];
        last_pos[5] = pos[5];
        return return_type::OK;
    }

    void Rebel::read()
    {
        pos[0] = currentStatus.posJointCurrent.at(0) * degToRad;
        pos[1] = currentStatus.posJointCurrent.at(1) * degToRad;
        pos[2] = currentStatus.posJointCurrent.at(2) * degToRad;
        pos[3] = currentStatus.posJointCurrent.at(3) * degToRad;
        pos[4] = currentStatus.posJointCurrent.at(4) * degToRad;
        pos[5] = currentStatus.posJointCurrent.at(5) * degToRad;
    }

    return_type Rebel::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Curently no use for time or period, here.
        write();
        return return_type::OK;
    }

    void Rebel::write()
    {
        // Check and call DIO callback
        if (rclcpp::ok())
        {
            rclcpp::spin_some(node_);
        }

        j1 = (float)vel_cmd[0];
        j2 = (float)vel_cmd[1];
        j3 = (float)vel_cmd[2];
        j4 = (float)vel_cmd[3];
        j5 = (float)vel_cmd[4];
        j6 = (float)vel_cmd[5];
    }

    void Rebel::dio_callback(
        const std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Request> request,
        std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Response> response)
    {
        SetDigitalOut(request->output.output, request->output.is_on);
        response->success = true;
    }

    void Rebel::GetReferenceInfo()
    {
        Command(std::string("GetReferencingInfo"));
    }

    void Rebel::Start()
    {
        continueMessage = true;
        messageThread = std::thread(&Rebel::MessageThreadFunction, this);

        rebelSocket->Start();

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Command(CriKeywords::COMMAND_CONNECT); // Gets a CMDERROR in CRI_V17
        Command(CriKeywords::COMMAND_SETACTIVE + " true");
        Command(CriKeywords::COMMAND_RESET);
        Command(CriKeywords::COMMAND_ENABLE);

        continueAlive = true;
        aliveThread = std::thread(&Rebel::AliveThreadFunction, this);

        GetConfig(CriKeywords::CONFIG_GETKINEMATICLIMITS);
        SetControlMode(ControlMode::VELOCITY);
    }

    void Rebel::Stop()
    {
        j1 = 0.0f;
        j2 = 0.0f;
        j3 = 0.0f;
        j4 = 0.0f;
        j5 = 0.0f;
        j6 = 0.0f;

        std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

        continueAlive = false;

        if (aliveThread.joinable())
        {
            aliveThread.join();
        }

        Command(CriKeywords::COMMAND_DISABLE);
        // Command(CriKeywords::COMMAND_DISCONNECT);
        Command(CriKeywords::COMMAND_QUIT);

        rebelSocket->Stop();

        continueMessage = false;

        if (messageThread.joinable())
        {
            messageThread.join();
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    Igus::Rebel, SystemInterface);
