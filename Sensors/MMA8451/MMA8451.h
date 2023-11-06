#include <Arduino.h>
#include "i2c_handler.h"

//#define DEBUG_ACC
#define INFO_ACC
#ifdef DEBUG_ACC
    #define INFO_ACC
#endif

#ifndef mmam8451_h
#define mmam8451_h

#define MMA8451_NONE 99
#define MMA8451_Z_AXIS 2
#define MMA8451_Y_AXIS 1
#define MMA8451_X_AXIS 0

class MMA8451{
    private:
        byte *_received_data;
        I2C_Handler _i2c;
        byte _f_mode;

        uint16_t _acc_sampling_rate;
        uint16_t _acc_scale;
        float _acc_sensitivity;

        int16_t _acc_offset[3]={0,0,0};

        byte _axis_aligned_index;
        bool _axis_aligned_direction;
        
    public:
        MMA8451(byte *received_data);
        ~MMA8451();
        bool check_who_am_i();
        void set_F_SETUP(byte F_MODE, byte F_WMRK);
        void get_DATA_STATUS();
        void set_gravity_compensation(byte axis_aligned_index, bool axis_aligned_direction);
        void set_offset_value_with_current_vals();
        void get_SYSMOD(byte FGERR, byte FGT, byte SYSMOD);
        void set_XYZ_DATA_CFG(byte HPF_OUT, byte FS);
        void set_CTRL_REG1(byte ASLP_RATE, byte DR, byte LNOISE, byte F_READ, byte ACTIVE);
        void set_CTRL_REG2(byte ST, byte RST, byte SLEEP, byte SLPE, byte MODS);
        void set_CTRL_REG3(byte FIFO_GATE, byte WAKE_TRANS, byte WAKE_LNDPRT, byte WAKE_PULSE, byte WAKE_FF_MT, byte IPOL, byte PP_OD);
        void set_CTRL_REG4(byte INT_EN_ASLP, byte INT_EN_FIFO, byte INT_EN_TRANS, byte INT_EN_LNDPRT, byte INT_EN_PULSE, byte INT_EN_FF_MT, byte INT_EN_DRDY);
        void set_CTRL_REG5(byte INT_CFG_ASLP, byte INT_CFG_FIFO, byte INT_CFG_TRANS, byte INT_CFG_LNDPRT, byte INT_CFG_PULSE, byte INT_CFG_FF_MT, byte INT_CFG_DRDY);

        void set_mode_1(uint8_t fifo_wtm_val, uint16_t sampling_rate, uint16_t scale);
        void set_sleep_mode_1();

        void get_INT_SOURCE();
        bool check_INT_SOURCE_FIFO();

        bool check_FIFO_WTM_status();
        bool check_FIFO_over_run_status();
        uint16_t check_FIFO_unread_samples();
        void get_acc_data(int16_t *fifo_data);
        void get_acc_data_raw(int16_t *fifo_data);

        byte get_sampling_rate_bin(uint16_t sampling_rate);
        byte get_scale_bin(uint16_t scale);

        uint16_t get_acc_data_sampling_rate();
        uint16_t get_acc_scale();
        uint16_t get_acc_sensitivity();

        typedef enum {
            DEVICE_ID = 0x1D,
            WHO_AM_I_VAL = 0x1A,
        } device_t;

        /** Registers
         *  
         * 0x00: F_STATUS - FIFO Status Register
         * 0x09: F_SETUP - FIFO Setup Register
         * 0x2D: CTRL_REG3 - Control Register 3
         * 0x2E: CTRL_REG4 - Control Register 4
        */
        typedef enum {
            DATA_STATUS = 0x00,
            OUT_X_MSB = 0x01,
            OUT_X_LSB = 0x02,
            OUT_Y_MSB = 0x03,
            OUT_Y_LSB = 0x04,
            OUT_Z_MSB = 0x05,
            OUT_Z_LSB = 0x06,
            F_SETUP = 0x09,
            SYSMOD = 0x0B,
            INT_SOURCE = 0x0C,
            WHO_AM_I = 0x0D,
            XYZ_DATA_CFG = 0x0E,
            CTRL_REG1 = 0x2A,
            CTRL_REG2 = 0x2B,
            CTRL_REG3 = 0x2C,
            CTRL_REG4 = 0x2D,
            CTRL_REG5 = 0x2E,
        } registers_t;

        /** FIFO Modes
         * 
         * FIFO_OFF: FIFO is disabled
         * FIFO_CIRCULAR: FIFO is in circular buffer mode
         * FIFO_FILL: FIFO is in fill mode
         * FIFO_TRIGGER: FIFO is in trigger mode
        */
        typedef enum {
            FIFO_OFF = 0b00,
            FIFO_CIRCULAR = 0b01,
            FIFO_FILL = 0b10,
            FIFO_TRIGGER = 0b11
        } fifo_mode_t;

        typedef enum {
            HPF_OUT_OFF = 0b0,
            HPF_OUT_ON = 0b1,
        } hp_filter_t;
        
        typedef enum {
            ASLP_RATE_50Hz = 0b000,
            ASLP_RATE_12_5Hz = 0b001,
            ASLP_RATE_6_25Hz = 0b010,
            ASLP_RATE_1_56Hz = 0b011,
        } aslp_rate_t;

        typedef enum{
            LNOISE_NORMAL = 0b0,
            LNOISE_LOW_NOISE = 0b1,
        } lnoise_t;

        typedef enum {
            F_READ_NORMAL = 0b0,
            F_READ_FAST = 0b1,
        } f_read_t;

        typedef enum {
            ACTIVE = 0b1,
            STANDBY = 0b0,
        } active_t;

        typedef enum{
            ST_DISBLE = 0b0,
            ST_ENABLE = 0b1,
        } st_t;
        
        typedef enum{
            RST_DISABLE = 0b0,
            RST_ENABLE = 0b1,
        } rst_t;

        typedef enum{
            SLEEPMODS_NORMAL = 0b00,
            SLEEPMODS_LOW_NOISE = 0b01,
            SLEEPMODS_HIGH_RES = 0b10,
            SLEEPMODS_LOW_POWER = 0b11,
        } sleepmods_t;

        typedef enum{
            SLPE_DISABLE = 0b0,
            SLPE_ENABLE = 0b1,
        } slpe_t;

        typedef enum{
            MODS_NORMAL = 0b00,
            MODS_LOW_NOISE = 0b01,
            MODS_HIGH_RES = 0b10,
            MODS_LOW_POWER = 0b11,
        } mods_t;

        typedef enum{
            FIFO_GATE_DISABLE = 0b0,
            FIFO_GATE_ENABLE = 0b1,
        } fifo_gate_t;
        
        typedef enum{
            WAKE_TRANS_SLEEP_MODE_DISABLE = 0b0,
            WAKE_TRANS_SLEEP_MODE_ENABLE = 0b1,
        } wake_trans_t;

        typedef enum{
            WAKE_LNDPRT_SLEEP_MODE_DISABLE = 0b0,
            WAKE_LNDPRT_SLEEP_MODE_ENABLE = 0b1,
        } wake_lndprt_t;

        typedef enum{
            WAKE_PULSE_SLEEP_MODE_DISABLE = 0b0,
            WAKE_PULSE_SLEEP_MODE_ENABLE = 0b1,
        } wake_pulse_t;

        typedef enum{
            WAKE_FF_MT_SLEEP_MODE_DISABLE = 0b0,
            WAKE_FF_MT_SLEEP_MODE_ENABLE = 0b1,
        } wake_ff_mt_t;

        typedef enum{
            IPOL_ACTIVE_LOW = 0b0,
            IPOL_ACTIVE_HIGH = 0b1,
        } ipol_t;

        typedef enum{
            PP_OD_PUSH_PULL = 0b0,
            PP_OD_OPEN_DRAIN = 0b1,
        } pp_od_t;

        typedef enum{
            INT_CFG_ASLP_INT1 = 0b1,
            INT_CFG_ASLP_INT2 = 0b0,
        } int_cfg_aslp_t;

        typedef enum{
            INT_CFG_FIFO_INT1 = 0b1,
            INT_CFG_FIFO_INT2 = 0b0,
        } int_cfg_fifo_t;

        typedef enum{
            INT_CFG_TRANS_INT1 = 0b0,
            INT_CFG_TRANS_INT2 = 0b1,
        } int_cfg_trans_t;

        typedef enum{
            INT_CFG_LNDPRT_INT1 = 0b1,
            INT_CFG_LNDPRT_INT2 = 0b0,
        } int_cfg_lndprt_t;

        typedef enum{
            INT_CFG_PULSE_INT1 = 0b1,
            INT_CFG_PULSE_INT2 = 0b0,
        } int_cfg_pulse_t;

        typedef enum{
            INT_CFG_FF_MT_INT1 = 0b1,
            INT_CFG_FF_MT_INT2 = 0b0,
        } int_cfg_ff_mt_t;

        typedef enum{
            INT_CFG_DRDY_INT1 = 0b1,
            INT_CFG_DRDY_INT2 = 0b0,
        } int_cfg_drdy_t;

        typedef enum{
            INT_EN_ASLP_ENABLE = 0b1,
            INT_EN_ASLP_DISABLE = 0b0,
        } int_en_aslp_t;

        typedef enum{
            INT_EN_FIFO_ENABLE = 0b1,
            INT_EN_FIFO_DISABLE = 0b0,
        } int_en_fifo_t;

        typedef enum{
            INT_EN_TRANS_ENABLE = 0b0,
            INT_EN_TRANS_DISABLE = 0b1,
        } int_en_trans_t;

        typedef enum{
            INT_EN_LNDPRT_ENABLE = 0b1,
            INT_EN_LNDPRT_DISABLE = 0b0,
        } int_en_lndprt_t;

        typedef enum{
            INT_EN_PULSE_ENABLE = 0b1,
            INT_EN_PULSE_DISABLE = 0b0,
        } int_en_pulse_t;

        typedef enum{
            INT_EN_FF_MT_ENABLE = 0b1,
            INT_EN_FF_MT_DISABLE = 0b0,
        } int_en_ff_mt_t;

        typedef enum{
            INT_EN_DRDY_ENABLE = 0b1,
            INT_EN_DRDY_DISABLE = 0b0,
        } int_en_drdy_t;

        typedef enum {
            AccScale_2G = 0b00,
            AccScale_4G = 0b01,
            AccScale_8G = 0b10,
        } mmam8451_scale_t;

        typedef enum {
            AccODR_800Hz = 0b000,
            AccODR_400Hz = 0b001,
            AccODR_200Hz = 0b010,
            AccODR_100Hz = 0b011,
            AccODR_50Hz = 0b100,
            AccODR_12_5Hz = 0b101,
            AccODR_6_25Hz = 0b110,
            AccODR_1_56Hz = 0b111,
        } mmam8451_odr_t;

};

#endif