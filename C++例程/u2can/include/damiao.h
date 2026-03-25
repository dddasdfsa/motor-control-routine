#ifndef DAMIAO_H
#define DAMIAO_H // DAMIAO_H 宏常量定义

#include "SerialPort.h" // 串口通信接口定义（USB转CAN串口）
#include <cmath> // 数学函数与浮点计算工具
#include <utility> // 通用工具（如 std::move）
#include <vector> // 动态数组容器支持
#include <unordered_map> // 哈希表容器支持
#include <array> // 定长数组容器支持
#include <variant> // 变体类型支持
#include <cstdint> // 定宽整数类型定义
#include <cmath> // 数学函数与浮点计算工具


#define POS_MODE 0x100 // 位置+速度控制报文ID偏移
#define SPEED_MODE 0x200 // 速度控制报文ID偏移
#define POSI_MODE 0x300 // 位置+电流控制报文ID偏移

#define POS_CSP_MODE 0x400 // CSP位置+速度控制报文ID偏移
#define SPEED_CSP_MODE 0x500 // CSP速度同步控制报文ID偏移
#define TOR_CSP_MODE 0x600 // CSP力矩同步控制报文ID偏移

#define max_retries 20 // 参数读写最大重试次数
#define retry_interval 50000 // 单次重试等待时间（微秒）
namespace damiao
{
#pragma pack(1) // 按1字节对齐结构体，匹配通信协议
#define Motor_id uint32_t // 电机ID类型别名（32位）

    /*!
     * @brief Motor Type 电机类型
     */
    enum DM_Motor_Type
    {
        DM4310, // DM4310 电机型号
        DM4310_48V, // DM4310_48V 电机型号
        DM4340, // DM4340 电机型号
        DM4340_48V, // DM4340_48V 电机型号
        DM6006, // DM6006 电机型号
        DM6248P, // DM6248P 电机型号
        DM8006, // DM8006 电机型号
        DM8009, // DM8009 电机型号
        DM10010L, // DM10010L 电机型号
        DM10010, // DM10010 电机型号
        DMH3510, // DMH3510 电机型号
        DMH6215, // DMH6215 电机型号
        DMG6220, // DMG6220 电机型号
        DMJH11, // DMJH11 电机型号
        Num_Of_Motor // 电机型号枚举数量
    };

    /*
     * @brief 电机控制模式
     */
    enum Control_Mode
    {//这是改控制模式对应的编码
        MIT_MODE=1,
        POS_VEL_MODE=2,
        VEL_MODE=3,
        POS_FORCE_MODE=4,

        POS_VEL_CSP_MODE=5, // CSP位置速度同步模式编码
        VEL_CSP_MODE=6, // CSP速度同步模式编码
        TORQUE_CSP_MODE=7, // CSP力矩同步模式编码
    };

    /*
     * @brief 寄存器列表 具体参考达妙手册
     */
    enum DM_REG
    {
        UV_Value = 0, // UV_Value 寄存器索引
        KT_Value = 1, // KT_Value 寄存器索引
        OT_Value = 2, // OT_Value 寄存器索引
        OC_Value = 3, // OC_Value 寄存器索引
        ACC = 4, // ACC 寄存器索引
        DEC = 5, // DEC 寄存器索引
        MAX_SPD = 6, // MAX_SPD 寄存器索引
        MST_ID = 7, // MST_ID 寄存器索引
        ESC_ID = 8, // ESC_ID 寄存器索引
        TIMEOUT = 9, // TIMEOUT 寄存器索引
        CTRL_MODE = 10, // CTRL_MODE 寄存器索引
        Damp = 11, // Damp 寄存器索引
        Inertia = 12, // Inertia 寄存器索引
        hw_ver = 13, // hw_ver 寄存器索引
        sw_ver = 14, // sw_ver 寄存器索引
        SN = 15, // SN 寄存器索引
        NPP = 16, // NPP 寄存器索引
        Rs = 17, // Rs 寄存器索引
        LS = 18, // LS 寄存器索引
        Flux = 19, // Flux 寄存器索引
        Gr = 20, // Gr 寄存器索引
        PMAX = 21, // PMAX 寄存器索引
        VMAX = 22, // VMAX 寄存器索引
        TMAX = 23, // TMAX 寄存器索引
        I_BW = 24, // I_BW 寄存器索引
        KP_ASR = 25, // KP_ASR 寄存器索引
        KI_ASR = 26, // KI_ASR 寄存器索引
        KP_APR = 27, // KP_APR 寄存器索引
        KI_APR = 28, // KI_APR 寄存器索引
        OV_Value = 29, // OV_Value 寄存器索引
        GREF = 30, // GREF 寄存器索引
        Deta = 31, // Deta 寄存器索引
        V_BW = 32, // V_BW 寄存器索引
        IQ_c1 = 33, // IQ_c1 寄存器索引
        VL_c1 = 34, // VL_c1 寄存器索引
        can_br = 35, // can_br 寄存器索引
        sub_ver = 36, // sub_ver 寄存器索引
        u_off = 50, // u_off 寄存器索引
        v_off = 51, // v_off 寄存器索引
        k1 = 52, // k1 寄存器索引
        k2 = 53, // k2 寄存器索引
        m_off = 54, // m_off 寄存器索引
        dir = 55, // dir 寄存器索引
        p_m = 80, // p_m 寄存器索引
        xout = 81, // xout 寄存器索引
    };

    typedef struct
    {
        uint8_t FrameHeader;
        uint8_t CMD;
        //     0x01: receive fail 0x11: receive success
        //     0x02: send fail 0x12: send success
        //     0x03: set baudrate fail 0x13: set baudrate success
        //     0xEE: communication error 此时格式段为错误码
        //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
        uint8_t canDataLen: 6;
        uint8_t canIde: 1;
        uint8_t canRtr: 1;
        uint32_t canId;
        uint8_t canData[8];
        uint8_t frameEnd;
    } CAN_Receive_Frame;

    typedef struct can_send_frame
    {
        uint8_t FrameHeader[2] = {0x55, 0xAA};
        uint8_t FrameLen = 0x1e;
        uint8_t CMD = 0x03;
        uint32_t sendTimes = 1;
        uint32_t timeInterval = 10;
        uint8_t IDType = 0;
        uint32_t canId=0x01;
        uint8_t frameType = 0;
        uint8_t len = 0x08;
        uint8_t idAcc=0;
        uint8_t dataAcc=0;
        uint8_t data[8]={0};
        uint8_t crc=0;

        void modify(const Motor_id id, const uint8_t* send_data)
        {
            canId = id;
            std::copy(send_data, send_data+8, data);
        }

    } can_send_frame;

#pragma pack() // 恢复默认结构体对齐方式

    typedef struct
    {
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    }Limit_param;

    //电机PMAX DQMAX TAUMAX参数
    Limit_param limit_param[Num_Of_Motor]= // 电机限制参数缓存
            {
                    {12.5, 30, 10 },
                    {12.5, 50, 10 },
                    {12.5, 8, 28 },
                    {12.5, 10, 28 },
                    {12.5, 45, 20 },
                    {12.566, 20, 120 },
                    {12.5, 45, 40 },
                    {12.5, 45, 54 },
                    {12.5,25,  200},
                    {12.5,20, 200},
                    {12.5,280,1},
                    {12.5,45,10},
                    {12.5,45,10} ,
                    {12.5,10,12}
            };

    class Motor
    {
    private:
        /* data */
        Motor_id Master_id;
        Motor_id Slave_id;
        float state_q=0;
        float state_dq=0;
        float state_tau=0;
        Limit_param limit_param{}; // 电机限制参数缓存
        DM_Motor_Type Motor_Type;

        union ValueUnion {
            float floatValue;
            uint32_t uint32Value;
        };

        struct ValueType {
            ValueUnion value;
            bool isFloat;
        };

        std::unordered_map<uint32_t , ValueType> param_map; // 寄存器参数缓存映射表
    public:
        /**
         * @brief Construct a new Motor object
         *
         * @param Motor_Type 电机类型
         * @param Slave_id canId 从机ID即电机ID
         * @param Master_id 主机ID建议主机ID不要都设为0x00
         *
         */
        Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id) // 构造电机对象并装载限制参数
                : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type) {
            this->limit_param = damiao::limit_param[Motor_Type];
        }

        Motor() : Master_id(0x01), Slave_id(0x11), Motor_Type(DM4310) { // 构造电机对象并装载限制参数
            this->limit_param = damiao::limit_param[DM4310];
        }

        void receive_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        DM_Motor_Type GetMotorType() const { return this->Motor_Type; }

        /*
         * @brief get master id 获取主机ID
         * @return MasterID
         */
        Motor_id GetMasterId() const { return this->Master_id; } // 获取主机ID

        /*
         * @brief get motor slave id(can id)  获取电机CAN ID
         * @return SlaveID
         */
        Motor_id GetSlaveId() const { return this->Slave_id; } // 获取从机（电机）ID

        /*
         * @brief get motor position 获取电机位置
         * @return motor position 电机位置
         */
        float Get_Position() const { return this->state_q; } // 获取电机位置状态

        /*
         * @brief get motor velocity 获取电机速度
         * @return motor velocity 电机速度
         */
        float Get_Velocity() const { return this->state_dq; } // 获取电机速度状态

        /*
         * @brief get torque of the motor  获取电机实际输出扭矩
         * @return motor torque 电机实际输出扭矩
         */
        float Get_tau() const { return this->state_tau; } // 获取电机力矩状态

        /*
         * @brief get limit param 获取电机限制参数
         * @return limit_param 电机限制参数
         */
        Limit_param get_limit_param() { return limit_param; } // 获取当前限制参数

        void set_param(int key, float value) // 缓存读取到的寄存器参数
        {
            ValueType v{};
            v.value.floatValue = value;
            v.isFloat = true;
            param_map[key] = v;
        }

        void set_param(int key, uint32_t value) // 缓存读取到的寄存器参数
        {
            ValueType v{};
            v.value.uint32Value = value;
            v.isFloat = false;
            param_map[key] = v;
        }

        float get_param_as_float(int key) const // 按float类型读取缓存参数
        {
            auto it = param_map.find(key);
            if (it != param_map.end())
            {
                if (it->second.isFloat)
                {
                    return it->second.value.floatValue;
                }
                else // 默认分支处理
                {
                    return 0;
                }
            }
            return 0;
        }

        uint32_t get_param_as_uint32(int key) const { // 按uint32类型读取缓存参数
            auto it = param_map.find(key);
            if (it != param_map.end()) {
                if (!it->second.isFloat) {
                    return it->second.value.uint32Value;
                }
                else // 默认分支处理
                {
                    return 0;
                }
            }
            return 0;
        }

        bool is_have_param(int key) const // 判断参数缓存中是否存在指定键
        {
            return param_map.find(key) != param_map.end();
        }

    };


/**
 * @brief motor control class 电机控制类
 *
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
    class Motor_Control
    {
    public:

        /*
        * @brief 定义电机控制对象
        * @param serial 串口对象
        * 默认串口为/dev/ttyACM0
        */
        Motor_Control(SerialPort::SharedPtr serial = nullptr): serial_(std::move(serial)) // 构造控制器并初始化串口
        {
            if (serial_ == nullptr) {
                //default serial port
                serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
            }
        }

        ~Motor_Control() // 析构函数（默认行为）
        = default;

        /*
        * @brief enable the motor 使能电机
        * @param motor 电机对象
        */
        void enable(const Motor& motor) // 发送使能指令并读取反馈
        {
            control_cmd(motor.GetSlaveId(), 0xFC); // 获取从机（电机）ID
            usleep(100000);
            this->receive();
        }

        /*
         * @brief enable motor which is old version 使能达妙旧款电机固件 使用旧版本固件建议尽快升级成新版本
         * @param motor object 电机对象
         * @param mode 控制模式  damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
         */
        void enable_old(const Motor& motor, Control_Mode mode)
        {
            uint32_t id = ((mode -1) << 2) + motor.GetSlaveId(); // 获取从机（电机）ID
            control_cmd(id, 0xFC);
            usleep(100000);
            this->receive();
        }

        /*
         * @brief refresh motor status 刷新电机状态
         * @param motor object 电机对象
         */
        void refresh_motor_status(const Motor& motor)
        {
            uint32_t id = 0x7FF;
            uint8_t can_low = motor.GetSlaveId() & 0xff; // id low 8 bit // 获取从机（电机）ID
            uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; //id high 8 bit // 获取从机（电机）ID
            std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00}; // 构造8字节CAN数据区
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }
        /*
        * @brief  disable the motor 失能电机
        * @param  motor object 电机对象
        */
        void disable(const Motor& motor) { // 发送失能指令并读取反馈
            control_cmd(motor.GetSlaveId(), 0xFD); // 获取从机（电机）ID
            usleep(100000);
            this->receive();
        }

        /*
        * @brief set the now position as zero point 保存当前位置为电机零点
        * @param motor object 电机对象
        */
        void set_zero_position(const Motor& motor) // 将当前位置设为零点
        {
            control_cmd(motor.GetSlaveId(), 0xFE); // 获取从机（电机）ID
            usleep(100000);
            this->receive();
        }

        /* @description: MIT Control Mode MIT控制模式 具体描述请参考达妙手册
         *@param kp: 比例系数
         *@param kd: 微分系数
         *@param q: position 位置
         *@param dq: velocity 速度
         *@param tau: torque 扭矩
        */
        void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau) // MIT模式：打包位宽映射后的控制量
        {
            // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };
            Motor_id id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found"); // 参数校验失败时抛出运行时异常
            }
            auto& m = motors[id];
            uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
            uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
            Limit_param limit_param_cmd = m->get_limit_param(); // 获取当前限制参数
            uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
            uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
            uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

            std::array<uint8_t, 8> data_buf{}; // 构造8字节CAN数据区
            data_buf[0] = (q_uint >> 8) & 0xff;
            data_buf[1] = q_uint & 0xff;
            data_buf[2] = dq_uint >> 4;
            data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
            data_buf[4] = kp_uint & 0xff;
            data_buf[5] = kd_uint >> 4;
            data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
            data_buf[7] = tau_uint & 0xff;

            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description: Position Control Mode with velocity  位置速度控制模式
         * @param pos: position 位置
         * @param vel: velocity 速度
         * @param DM_Motor: Motor object 电机对象
         */
        void control_pos_vel(Motor &DM_Motor,float pos,float vel) // 位置速度模式：发送pos+vel
        {
            Motor_id id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf{}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &pos, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            memcpy(data_buf.data() + 4, &vel, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            id += POS_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description: velocity control mode 速度控制模式
         * @param DM_Motor: motor object 电机对象
         * @param vel: velocity 速度
         */
        void control_vel(Motor &DM_Motor,float vel) // 速度模式：发送vel
        {
            Motor_id id =DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("VEL ERROR : id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf = {0}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &vel, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            id=id+SPEED_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description: Position control mode with torque  力位混合控制模式
         * @param DM_Motor: motor object 电机对象
         * @param pos: position 位置
         * @param vel: velocity 速度 范围0-10000 具体参考达妙手册
         * @param i: current 电流 范围0-10000具体参考达妙手册
         */
        void control_pos_force(Motor &DM_Motor,float pos, uint16_t vel, uint16_t i) // 力位混合模式：发送pos+vel+i
        {
            Motor_id id =DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("pos_force ERROR : Motor_Control id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf{}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &pos, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t)); // 按字节拷贝数据到报文缓冲区
            memcpy(data_buf.data() + 6, &i, sizeof(uint16_t)); // 按字节拷贝数据到报文缓冲区
            id=id+POSI_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }


        /*
         * @description: Position Control Mode with velocity  周期同步位置速度控制模式
         * @param pos: position 位置
         * @param vel: velocity 速度
         * @param DM_Motor: Motor object 电机对象
         */
        void control_pos_vel_csp(Motor &DM_Motor,float pos,float vel) // CSP位置速度同步模式下发
        {
            Motor_id id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("POS_VEL_CSP ERROR : Motor_Control id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf{}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &pos, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            memcpy(data_buf.data() + 4, &vel, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            id += POS_CSP_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description: velocity control mode 周期同步速度控制模式
         * @param DM_Motor: motor object 电机对象
         * @param vel: velocity 速度
         */
        void control_vel_csp(Motor &DM_Motor,float vel) // CSP速度同步模式下发
        {
            Motor_id id =DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("VEL ERROR : id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf = {0}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &vel, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            id=id+SPEED_CSP_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description: torque control mode 周期同步力矩控制模式
         * @param DM_Motor: motor object 电机对象
         * @param tor: torque 力矩
         */
        void control_tor_csp(Motor &DM_Motor,float tor) // CSP力矩同步模式下发
        {
            Motor_id id =DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("VEL ERROR : id not found"); // 参数校验失败时抛出运行时异常
            }
            std::array<uint8_t, 8> data_buf = {0}; // 构造8字节CAN数据区
            memcpy(data_buf.data(), &tor, sizeof(float)); // 按字节拷贝数据到报文缓冲区
            id=id+TOR_CSP_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            this->receive();
        }

        /*
         * @description 函数库内部调用,用于解算电机can线返回的电机参数
         */
        void receive() // 解析实时状态反馈报文
        {
            serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame)); // 从串口接收CAN反馈帧

            if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55)
            {
                static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                    float span = xmax - xmin;
                    float data_norm = float(x) / ((1 << bits) - 1);
                    float data = data_norm * span + xmin;
                    return data;
                };

                auto & data = receive_data.canData;

                uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
                if(receive_data.canId != 0x00)
                {
                    if(motors.find(receive_data.canId) == motors.end())
                    {
                        return;
                    }

                    auto m = motors[receive_data.canId];
                    Limit_param limit_param_receive = m->get_limit_param(); // 获取当前限制参数
                    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                    m->receive_data(receive_q, receive_dq, receive_tau);
                }
                else //why the user set the masterid as 0x00 ??? // 默认分支处理
                {
                    uint32_t slaveID = data[0] & 0x0f;
                    if(motors.find(slaveID) == motors.end())
                    {
                        return;
                    }
                    auto m = motors[slaveID];
                    Limit_param limit_param_receive = m->get_limit_param(); // 获取当前限制参数
                    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                    m->receive_data(receive_q, receive_dq, receive_tau);
                }
                return;
            }
            else if (receive_data.CMD == 0x01)
            {
                /* code */
            }
            else if (receive_data.CMD == 0x02)
            {
                /* code */
            }
            else if (receive_data.CMD == 0x03)
            {
                /* code */
            }
            else if (receive_data.CMD == 0xEE)
            {
                /* code */
            }
        }

        void receive_param() // 解析寄存器读写反馈报文
        {
            serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame)); // 从串口接收CAN反馈帧

            if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55)
            {
                auto & data = receive_data.canData;
                if(data[2]==0x33 or data[2]==0x55)
                {
                    uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                    uint8_t RID = data[3];
                    if (motors.find(slaveID) == motors.end())
                    {
                        //can not found motor id
                        return;
                    }
                    if(is_in_ranges(RID))
                    {
                        uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                        motors[slaveID]->set_param(RID, data_uint32); // 访问已注册电机对象
                    }
                    else // 默认分支处理
                    {
                        float data_float = uint8_to_float(data + 4);
                        motors[slaveID]->set_param(RID, data_float); // 访问已注册电机对象
                    }
                }
                return ;
            }
        }

        /**
         * @brief add motor to class 添加电机
         * @param DM_Motor : motor object 电机对象
         */
        void addMotor(Motor *DM_Motor) // 注册电机对象到内部映射
        {
            motors.insert({DM_Motor->GetSlaveId(), DM_Motor}); // 获取从机（电机）ID
            if (DM_Motor->GetMasterId() != 0)
            {
                motors.insert({DM_Motor->GetMasterId(), DM_Motor}); // 获取主机ID
            }
        }

        /*
         * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
         * @param DM_Motor: motor object 电机对象
         * @param RID: register id 寄存器ID  example: damiao::UV_Value
         * @return: motor param 电机参数 如果没查询到返回的参数为0
         */
        float read_motor_param(Motor &DM_Motor,uint8_t RID) // 读取指定寄存器并等待返回值
        {
            uint32_t id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00}; // 构造8字节CAN数据区
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            for(uint8_t i =0;i<max_retries;i++) // 重试循环处理
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
                    }
                    else // 默认分支处理
                    {
                        return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
                    }
                }
            }

            return 0;
        }


        /*
         * @description: switch control mode 切换电机控制模式
         * @param DM_Motor: motor object 电机对象
         * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
         */
        bool switchControlMode(Motor &DM_Motor,Control_Mode mode) // 写入控制模式并校验是否生效
        {
            uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
            uint8_t RID = 10;
            write_motor_param(DM_Motor,RID,write_data);
            if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            for(uint8_t i =0;i<max_retries;i++) // 重试循环处理
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
                }
            }
            return false;
        }

        /*
         * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
         * @param DM_Motor: motor object 电机对象
         * @param RID: register id 寄存器ID
         * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
         * @return: bool true or false  是否修改成功
         */
        bool change_motor_param(Motor &DM_Motor,uint8_t RID,float data) // 写寄存器并轮询确认写入成功
        {
            if(is_in_ranges(RID)) {
                //居然传进来的是整型的范围 救一下
                uint32_t data_uint32 = float_to_uint32(data);
                uint8_t *data_uint8;
                data_uint8=(uint8_t*)&data_uint32;
                write_motor_param(DM_Motor,RID,data_uint8);
            }
            else // 默认分支处理
            {
                //is float
                uint8_t *data_uint8;
                data_uint8=(uint8_t*)&data;
                write_motor_param(DM_Motor,RID,data_uint8);
            }
            if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            for(uint8_t i =0;i<max_retries;i++) // 重试循环处理
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
                    }
                    else // 默认分支处理
                    {
                        return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) - data)<0.1f;
                    }
                }
            }
            return false;
        }


        /*
         * @description: save all param to motor flash 保存电机的所有参数到flash里面
         * @param DM_Motor: motor object 电机对象
         * 电机默认参数不会写到flash里面，需要进行写操作
         */
        void save_motor_param(Motor &DM_Motor) // 将当前参数保存到电机Flash
        {
            disable(DM_Motor);
            uint32_t id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            uint8_t id_low = id & 0xff;
            uint8_t id_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00}; // 构造8字节CAN数据区
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
            usleep(100000);
        }

        /*
         * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
         * @param DM_Motor: motor object 电机对象
         * @param P_MAX: position max 位置最大值
         * @param Q_MAX: velocity max 速度最大值
         * @param T_MAX: torque max 扭矩最大值
         */
        static void changeMotorLimit(Motor &DM_Motor,float P_MAX,float Q_MAX,float T_MAX) // 修改库内电机限制参数表
        {
            limit_param[DM_Motor.GetMotorType()]={P_MAX,Q_MAX,T_MAX};
        }

        /*
         * @description: change motor PMAX 修改电机的最大PMAX
         * @param DM_Motor: motor object 电机对象
         * @param P_MAX: position max 位置最大值
         */
        static void changeMotorPMAX(Motor &DM_Motor,float P_MAX) // 修改库内位置上限参数
        {
            limit_param[DM_Motor.GetMotorType()].Q_MAX=P_MAX;
        }

    private:
        void control_cmd(Motor_id id , uint8_t cmd) // 封装并发送通用控制命令帧
        {
            std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd}; // 构造8字节CAN数据区
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
        }

        void write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4]) // 封装并发送寄存器写入帧
        {
            uint32_t id = DM_Motor.GetSlaveId(); // 获取从机（电机）ID
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00}; // 构造8字节CAN数据区
            data_buf[4] = data[0];
            data_buf[5] = data[1];
            data_buf[6] = data[2];
            data_buf[7] = data[3];
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame)); // 通过串口下发CAN转发帧
        }

        static bool is_in_ranges(int number) { // 判断寄存器是否为整型参数区间
            return (7 <= number && number <= 10) ||
                   (13 <= number && number <= 16) ||
                   (35 <= number && number <= 36);
        }

        static uint32_t float_to_uint32(float value) { // 浮点数转无符号整型（截断）
            return static_cast<uint32_t>(value);
        }

        static float uint32_to_float(uint32_t value) { // 无符号整型转浮点数
            return static_cast<float>(value);
        }

        static float uint8_to_float(const uint8_t data[4]) { // 4字节数组按小端还原float
            uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                                (static_cast<uint32_t>(data[2]) << 16) |
                                (static_cast<uint32_t>(data[1]) << 8)  |
                                static_cast<uint32_t>(data[0]);
            float result;
            memcpy(&result, &combined, sizeof(result)); // 按字节拷贝数据到报文缓冲区
            return result;
        }

        std::unordered_map<Motor_id, Motor*> motors; // 电机ID到对象指针的映射表
        SerialPort::SharedPtr serial_;  //serial port // 串口对象句柄
        can_send_frame send_data; //send data frame // 发送帧缓存
        CAN_Receive_Frame receive_data{};//receive data frame // 接收帧缓存
    };

};

#endif
