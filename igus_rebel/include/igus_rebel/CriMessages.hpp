#ifndef CRI_MESSAGE_HPP_
#define CRI_MESSAGE_HPP_

#include <array>
#include <vector>
#include <regex>

// Classes to parse the messages the robot may send and to store the parse result.
// Each message type has a constructor that takes a message string as sent from the robot
// (without the CRISTART and CRIEND) which then parses the string and saves what it finds
// in the class members.
//
// There is a base class (CriMessage) which constains a static helper method
// (CriMessage::GetMessageType) to determine a message's type.
//
// So to process a message, you first call CriMessage::GetMessageType and then construct
// the appropriate message type.

namespace Igus
{
    namespace CriMessages
    {
        enum class MessageType
        {
            STATUS,
            OPINFO,
            GSIG,
            GRIPPERSTATE,
            RUNSTATE,
            MESSAGE,
            CMD,
            CONFIG,
            INFO,
            LOGMSG,
            VARIABLES,
            CMDERROR,
            CMDACK,
            CYCLESTAT,
            UNKNOWN = 1000
        };

        enum class Mode
        {
            JOINT,
            CARTBASE,
            CARTTOOL,
            PLATFORM,
            FSM,
            UNKNOWN = 1000
        };

        enum class Kinstate
        {
            NO_ERROR = 0,
            JOINT_LIMIT_MIN = 13,
            JOINT_LIMIT_MAX = 14,
            CARTESIAN_SINGULARITY_CENTER = 21,
            CARTESIAN_SINGULARITY_REACH = 23,
            CARTESIAN_SINGULARITY_WRIST = 24,
            TOOL_AT_VIRTUAL_BOX_LIMIT_1 = 30,
            TOOL_AT_VIRTUAL_BOX_LIMIT_2 = 31,
            TOOL_AT_VIRTUAL_BOX_LIMIT_3 = 32,
            TOOL_AT_VIRTUAL_BOX_LIMIT_4 = 33,
            TOOL_AT_VIRTUAL_BOX_LIMIT_5 = 34,
            TOOL_AT_VIRTUAL_BOX_LIMIT_6 = 35,
            MOTION_NOT_ALLOWED = 99,
            UNKNOWN = 1000
        };

        enum class ErrorJoint
        {
            TEMP = 1,       // OverTemperature
            ESTOP_LOWV = 2, // Energy supply too low
            MNE = 4,        // Motor not enabled
            COM = 8,        // Communication watch dog
            POS = 16,       // Position lag
            ENC = 32,       // Encoder Error
            OC = 64,        // Overcurrent
            DRV = 128       // Driver Error
        };

        enum class ConfigType
        {
            KINEMATICLIMITS,
            UNKNOWN = 1000
        };

        enum class LogLevel
        {
            DEBUG,
            APP_INFO,
            APP_ERROR,
            INFO,
            WARN,
            ERROR,
            FATAL,
            UNKNOWN = 1000
        };

        class CriMessage
        {
        public:
            MessageType GetType()
            {
                return type;
            }

            static MessageType GetMessageType(const std::string &);

        protected:
            MessageType type;
            explicit CriMessage(const MessageType &_type) : type(_type) {}

            static std::string ParseMessageString(const std::string &, const std::string::size_type &,
                                                  const std::string::size_type &, const std::string::size_type &);

            template <class T, std::size_t N>
            static void FillArray(std::array<T, N> &, const std::string &);

            template <class T>
            void FillVector(std::vector<T> &, const std::string &);

            template <class T, std::size_t N>
            static std::string ArrayToString(std::array<T, N> &);

            template <class T>
            std::string VectorToString(std::vector<T> &vector);
        };

        class Status : public CriMessage
        {
        public:
            Mode mode;
            std::array<float, 16> posJointSetPoint;
            std::array<float, 16> posJointCurrent;
            std::array<float, 6> posCartRobot;
            std::array<float, 3> posCartPlattform;
            float overrideValue;
            int din;
            int dout;
            int eStop;
            int supply;
            int currentall;
            std::array<int, 16> currentjoints;
            std::string errorSummary;
            std::array<int, 16> errorJoints;
            Kinstate kinstate;

            explicit Status(const std::string &);
            explicit Status();
            static std::string ModeToString(const Mode &);
            void Print();
            void Log();

        private:
            std::string ToString();
            Mode GetMode(const std::string &);
            Kinstate GetKinstate(const std::string &);
        };

        class Message : public CriMessage
        {
        public:
            explicit Message(const std::string &);

            std::string message;
        };

        class Command : public CriMessage
        {
        public:
            explicit Command(const std::string &);

            std::string command;
        };

        class Info : public CriMessage
        {
        public:
            explicit Info(const std::string &);

            std::string info;
        };

        class LogMsg : public CriMessage
        {
        public:
            explicit LogMsg(const std::string &);

            LogLevel logLevel;
            long timestamp;
            std::string logMsg;

        private:
            LogLevel GetLogLevel(const std::string &);
        };

        class Variables : public CriMessage
        {
        public:
            explicit Variables(const std::string &);

            struct NumberVariable
            {
                std::string name;
                double value;
            };

            struct PositionVariable
            {
                std::string name;
                std::array<double, 15> values;
            };

            std::vector<NumberVariable> numberVariables;
            std::vector<PositionVariable> positionVariables;

        private:
            std::string GetMatchString(const std::smatch &);
        };

        class CmdError : public CriMessage
        {
        public:
            explicit CmdError(const std::string &);
            int recjectedCmd;
            std::string error;
        };

        class CmdAck : public CriMessage
        {
        public:
            explicit CmdAck(const std::string &);
            int acceptedCmd;
        };

        class Cyclestat : public CriMessage
        {
        public:
            explicit Cyclestat(const std::string &);
            int cycletime;
            int workload;
        };

        class Config : public CriMessage
        {
        public:
            ConfigType configType;
            static ConfigType GetConfigType(const std::string &);

        protected:
            explicit Config(const ConfigType &_configType) : CriMessage(MessageType::CONFIG), configType(_configType) {}
        };

        class KinematicLimits : public Config
        {
        public:
            explicit KinematicLimits(const std::string &);

            // We are using a vector here, instead of a fixed size array, because the documentation
            // is unclear on the question if the robot will always return the same number of paris.
            // (Probably it will, but why risk it?)
            std::vector<std::pair<float, float>> minMaxPairs;

            std::string ToString();
            void Print();
        };
    }
}

#endif
