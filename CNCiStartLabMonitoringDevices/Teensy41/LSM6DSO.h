#include "i2c_handler.h"
#include "spi_handler.h"

#define DEVICE_ID 0x6C
#define FIFO_MAX_SAMPLES 2048

#define I2C_com 0x00
#define SPI_com 0x01

//#define DEBUG_LSM6DSO
//#define INFO_LSM6DSO

#ifdef DEBUG_LSM6DSO
    #define INFO_LSM6DSO
#endif

#ifndef lsm6dso_h
#define lsm6dso_h

class LSM6DSO{
    private:

        byte *_received_data;
        byte _communication_protocol;
        I2C_Handler _i2c;
        SPI_Handler _spi;
        byte _who_am_i_val;

        uint16_t _acc_sampling_rate;
        uint16_t _acc_scale;
        float _acc_sensitivity;

    public:

        LSM6DSO(byte *received_data);
        ~LSM6DSO();
        void begin(byte communication_protocol);
        void begin(byte communication_protocol, int SS_pin);
        void begin(byte communication_protocol, int SS_pin, byte WHO_AM_I_VAL);
        bool check_who_am_i(); 
        void set_mode_1(uint16_t acc_fifo_wtm_val);
        void set_F_SETUP(byte F_MODE, byte F_WMRK);
        void set_FIFO_CTRL1(uint16_t WTM);
        void set_FIFO_CTRL2(byte STOP_ON_WTM, byte FIFO_COMPR_RT_EN, byte ODRCHG_EN, byte UNCOPTR_RATE, uint16_t WTM_8);
        void set_FIFO_CTRL3(byte BDR_GY, byte BDR_XL);
        void set_FIFO_CTRL4(byte DEC_TS_BATCH ,byte ODR_T_BATCH , byte FIFO_MODE);
        void set_COUNTER_BDR_REG1(byte datareeady_pulsed, byte RST_COUNTER_BDR, byte TRIG_COUNTER_BDR, byte CNT_BDR_TH);
        void set_COUNTER_BDR_REG2(byte CNT_BDR_TH);
        void set_INT1_CTRL(byte DEN_DRDY, byte INT1_CNT_BDR, byte INT1_FIFO_FULL, byte INT1_FIFO_OVR, byte INT1_FIFO_TH, byte INT1_BOOT, byte INT1_DRDY_G, byte INT1_DRDY_XL);
        void set_INT2_CTRL(byte INT2_CTN_BDR, byte INT2_FIFO_FULL, byte INT2_FIFO_OVR, byte INT2_FIFO_TH, byte INT2_DRDY_TEMP, byte INT2_DRDY_G, byte INT2_DRDY_XL);
        void set_CTRL1_XL(byte ODR_XL, byte FS1_XL, byte LPF2_XL_EN);
        void set_CTRL3_C(byte BOOT, byte BDU, byte H_LACTIVE, byte PP_OD, byte SIM, byte IF_INC, byte SW_RESET);
        void set_CTRL4_C(byte SLEEP_G, byte INT2_on_INT1, byte DRDY_MASK, byte I2C_disable, byte LPF1_SEL_G);
        void set_CTRL5_C(byte XL_ULP_EN, byte ROUNDING, byte ST_G, byte ST_XL);
        void set_CTRL8_XL(byte HPCF_XL_2, byte HP_REF_MODE, byte FASTSETTL_MODE_XL, byte HP_SLOPE_XL_EN, byte XL_FS_MODE, byte LOW_PASS_ON_6D);
        void get_STATUS_REG();
        bool check_XLDA();
        void get_OUTX_A(int16_t *data);
        void get_OUTX_A_raw(int16_t *data);
        void get_FIFO_STATUS();
        bool check_FIFO_OVR();
        bool check_FIFO_FULL();
        bool check_FIFO_WTM();
        uint16_t check_FIFO_unread_samples();
        void get_FIFO_DATA_OUT(int16_t *data);
        void get_FIFO_DATA_OUT_raw(int16_t *data);
        uint16_t get_acc_data_sampling_rate();
        uint16_t get_acc_scale();
        uint16_t get_acc_sensitivity();

        typedef enum{
            FIFO_CTRL1 = 0x07,
            FIFO_CTRL2 = 0x08,
            FIFO_CTRL3 = 0x09,
            FIFO_CTRL4 = 0x0A,
            COUNTER_BDR_REG1 = 0x0B,
            COUNTER_BDR_REG2 = 0x0C,
            INT1_CTRL = 0x0D,
            INT2_CTRL = 0x0E,
            WHO_AM_I = 0x0F,
            CTRL1_XL = 0x10,
            CTRL2_G = 0x11,
            CTRL3_C = 0x12,
            CTRL4_C = 0x13,
            CTRL5_C = 0x14,
            CTRL6_C = 0x15,
            CTRL7_G = 0x16,
            CTRL8_XL = 0x17,
            CTRL9_XL = 0x18,
            CTRL10_C = 0x19,
            STATUS_REG = 0x1E,
            OUTX_L_A = 0x28,
            OUTX_H_A = 0x29,
            OUTY_L_A = 0x2A,
            OUTY_H_A = 0x2B,
            OUTZ_L_A = 0x2C,
            OUT_Z_H_A = 0x2D,
            FIFO_STATUS1 = 0x3A,
            FIFO_STATUS2 = 0x3B,
            FIFO_DATA_OUT_TAG = 0x78,
            FIFO_DATA_OUT_X_L = 0x79,
            FIFO_DATA_OUT_X_H = 0x7A,
            FIFO_DATA_OUT_Y_L = 0x7B,
            FIFO_DATA_OUT_Y_H = 0x7C,
            FIFO_DATA_OUT_Z_L = 0x7D,
            FIFO_DATA_OUT_Z_H = 0x7E,

        }lsm6dso_registers_t;

        typedef enum{
            FIFO_ODR_XL_12_5Hz = 0x01,
            FIFO_ODR_XL_26Hz = 0x02,
            FIFO_ODR_XL_52Hz = 0x03,
            FIFO_ODR_XL_104Hz = 0x04,
            FIFO_ODR_XL_208Hz = 0x05,
            FIFO_ODR_XL_417Hz = 0x06,
            FIFO_ODR_XL_833Hz = 0x07,
            FIFO_ODR_XL_1667Hz = 0x08,
            FIFO_ODR_XL_3333Hz = 0x09,
            FIFO_ODR_XL_6667Hz = 0x0A,
            FIFO_ODR_XL_1_6Hz = 0x0B,
        }lsm6dso_fifo_odr_xl;

        typedef enum{
            STOP_ON_WTM_DISABLED = 0x00,
            STOP_ON_WTM_ENABLED = 0x01,
        }lsm6dso_stop_on_wtm_t;

        typedef enum{
            FIFO_COMPR_RT_DISABLED = 0x00,
            FIFO_COMPR_RT_ENABLED = 0x01,
        }lsm6dso_fifo_compr_rt_t;

        typedef enum{
            ODRCHG_DISABLED = 0x00,
            ODRCHG_ENABLED = 0x01,
        }lsm6dso_odrchg_t;

        typedef enum{
            UNCOPTR_RATE_DISABLED = 0x00,
            UNCOPTR_RATE_ENABLED = 0x01,
        }lsm6dso_uncoptr_rate_t;

        typedef enum{
            FIFO_BDR_GY_DISABLED = 0x00,
            FIFO_BDR_GY_12_5Hz = 0x01,
            FIFO_BDR_GY_26Hz = 0x02,
            FIFO_BDR_GY_52Hz = 0x03,
            FIFO_BDR_GY_104Hz = 0x04,
            FIFO_BDR_GY_208Hz = 0x05,
            FIFO_BDR_GY_417Hz = 0x06,
            FIFO_BDR_GY_833Hz = 0x07,
            FIFO_BDR_GY_1667Hz = 0x08,
            FIFO_BDR_GY_3333Hz = 0x09,
            FIFO_BDR_GY_6500Hz = 0x0A,
        }lsm6dso_fifo_bdr_gy_t;

        typedef enum{
            FIFO_BDR_XL_DISABLED = 0x00,
            FIFO_BDR_XL_12_5Hz = 0x01,
            FIFO_BDR_XL_26Hz = 0x02,
            FIFO_BDR_XL_52Hz = 0x03,
            FIFO_BDR_XL_104Hz = 0x04,
            FIFO_BDR_XL_208Hz = 0x05,
            FIFO_BDR_XL_417Hz = 0x06,
            FIFO_BDR_XL_833Hz = 0x07,
            FIFO_BDR_XL_1667Hz = 0x08,
            FIFO_BDR_XL_3333Hz = 0x09,
            FIFO_BDR_XL_6667Hz = 0x0A,
        }lsm6dso_fifo_bdr_xl_t;

        typedef enum{
            DEC_TS_BATCH_DISABLED = 0x00,
            DEC_TS_BATCH_1 = 0x01,
            DEC_TS_BATCH_8 = 0x02,
            DEC_TS_BATCH_32 = 0x03,
        }lsm6dso_dec_ts_batch_t;

        typedef enum{
            ODR_T_BATCH_DISABLED = 0x00,
            ODR_T_BATCH_1_6Hz = 0x01,
            ODR_T_BATCH_12_5Hz = 0x02,   
            ODR_T_BATCH_52Hz = 0x03,
        } lsm6dso_odr_t_batch_t;

        typedef enum{
            FIFO_MODE_DISABLED = 0x00,
            FIFO_MODE_ENABLED = 0x01,
            FIFO_MODE_CONTINUOUS_FIFO = 0x03,
            FIFO_MODE_BYPASS_CONTINUOUS = 0x04,
            FIFO_MODE_CONTINUOUS = 0x06,
            FIFO_MODE_BYPASS_FIFO = 0x07,
        }lsm6dso_fifo_mode_t;

        typedef enum{
            XL_ODR_12_5Hz = 0x01,
            XL_ODR_26Hz = 0x02,
            XL_ODR_52Hz = 0x03,
            XL_ODR_104Hz = 0x04,
            XL_ODR_208Hz = 0x05,
            XL_ODR_417Hz = 0x06,
            XL_ODR_833Hz = 0x07,
            XL_ODR_1666Hz = 0x08,
            XL_ODR_3333Hz = 0x09,
            XL_ODR_6666Hz = 0x0A,
            XL_ODR_1_6Hz = 0x0B,
        }lsm6dso_xl_odr_t;

        typedef enum{
            XL_FS_2g = 0x00,
            XL_FS_16g_2g = 0x01,
            XL_FS_4g = 0x02,
            XL_FS_8g = 0x03,
        }lsm6dso_xl_fs_t;

        typedef enum{
            LPF2_XL_EN_DISABLED = 0x00,
            LPF2_XL_EN_ENABLED = 0x01,
        }lsm6dso_lpf2_xl_en_t;

        typedef enum{
            BOOT_NORMAL_MODE = 0x00,
            BOOT_REBOOT_MODE = 0x01,
        }lsm6dso_boot_t;

        typedef enum{
            BDU_CONTINUOS_UPDATE = 0x00,
            BDU_BLOCK_UPDATE = 0x01,
        }lsm6dso_bdu_t;

        typedef enum{
            H_LACTIVE_HIGH = 0x00,
            H_LACTIVE_LOW = 0x01,
        }lsm6dso_h_lactive_t;

        typedef enum{
            PP_OD_PUSH_PULL = 0x00,
            PP_OD_OPEN_DRAIN = 0x01,
        }lsm6dso_pp_od_t;

        typedef enum{
            SIM_4WIRE = 0x00,
            SIM_3WIRE = 0x01,
        }lsm6dso_sim_t;

        typedef enum{
            IF_INC_DISABLED = 0x00,
            IF_INC_ENABLED = 0x01,
        }lsm6dso_if_inc_t;

        typedef enum{
            SW_RESET_NORMAL_MODE = 0x00,
            SW_RESET_RESET_DEVICE = 0x01,
        }lsm6dso_sw_reset_t;

        typedef enum{
            SLEEP_G_DISABLED = 0x00,
            SLEEP_G_ENABLED = 0x01,
        }lsm6dso_sleep_g_t;

        typedef enum{
            INT2_on_INT1_DISABLED = 0x00,
            INT2_on_INT1_ENABLED = 0x01,
        }lsm6dso_int2_on_int1_t;

        typedef enum{
            DRDY_MASK_DISABLED = 0x00,
            DRDY_MASK_ENABLED = 0x01,   
        }lsm6dso_drdy_mask_t;

        typedef enum{
            I2C_DISABLE_DISABLED = 0x00,
            I2C_DISABLE_ENABLED = 0x01,
        }lsm6dso_i2c_disable_t;

        typedef enum{
            LPF1_SEL_G_DISABLED = 0x00,
            LPF1_SEL_G_ENABLED = 0x01,
        }lsm6dso_lpf1_sel_g_t;

        typedef enum{
            XL_ULP_EN_DISABLED = 0x00,
            XL_ULP_EN_ENABLED = 0x01,
        }lsm6dso_xl_ulp_en_t;

        typedef enum{
            ROUNDING_DISABLED = 0x00,
            ROUNDING_XL_ENABLED = 0x01,
            ROUNDING_G_ENABLED = 0x02,
            ROUNDING_XL_G_ENABLED = 0x03,
        }lsm6dso_rounding_t;

        typedef enum{
            ST_G_DISABLED = 0x00,
            ST_G_ENABLED = 0x01,
        }lsm6dso_st_g_t;

        typedef enum{
            ST_XL_DISABLED = 0x00,
            ST_XL_ENABLED = 0x01,
        }lsm6dso_st_xl_t;

        typedef enum{
            HPCF_XL_2_CONFIG1 = 0x00,
            HPCF_XL_2_CONFIG2 = 0x01,
            HPCF_XL_2_CONFIG3 = 0x02,
            HPCF_XL_2_CONFIG4 = 0x03,
        }lsm6dso_hpcf_xl_2_t;

        typedef enum{
            HP_REF_MODE_DISABLED = 0x00,
            HP_REF_MODE_ENABLED = 0x01,
        }lsm6dso_hp_ref_mode_t;

        typedef enum{
            FASTSETTL_MODE_XL_DISABLED = 0x00,
            FASTSETTL_MODE_XL_ENABLED = 0x01,
        }lsm6dso_fastsettl_mode_xl_t;

        typedef enum{
            HP_SLOPE_XL_EN_DISABLED = 0x00,
            HP_SLOPE_XL_EN_ENABLED = 0x01,
        }lsm6dso_hp_slope_xl_en_t;

        typedef enum{
            XL_FS_MODE_DISABLED = 0x00,
            XL_FS_MODE_ENABLED = 0x01,
        }lsm6dso_xl_fs_mode_t;   

        typedef enum{
            LOW_PASS_ON_6D_DISABLED = 0x00,
            LOW_PASS_ON_6D_ENABLED = 0x01,
        }lsm6dso_low_pass_on_6d_t; 
    
        typedef enum{
            FIFO_ODR_G_12_5Hz = 0x01,
            FIFO_ODR_G_26Hz = 0x02,
            FIFO_ODR_G_52Hz = 0x03,
            FIFO_ODR_G_104Hz = 0x04,
            FIFO_ODR_G_208Hz = 0x05,
            FIFO_ODR_G_417Hz = 0x06,
            FIFO_ODR_G_833Hz = 0x07,
            FIFO_ODR_G_1667Hz = 0x08,
            FIFO_ODR_G_3333Hz = 0x09,
            FIFO_ODR_G_6667Hz = 0x0A,
            FIFO_ODR_G_6_5Hz = 0x0B,
        }lsm6dso_fifo_odr_g;

        typedef enum{
            G_ODR_12_5Hz = 0x01,
            G_ODR_26Hz = 0x02,
            G_ODR_52Hz = 0x03,
            G_ODR_104Hz = 0x04,
            G_ODR_208Hz = 0x05,
            G_ODR_417Hz = 0x06,
            G_ODR_833Hz = 0x07,
            G_ODR_1666Hz = 0x08,
            G_ODR_3333Hz = 0x09,
            G_ODR_6666Hz = 0x0A,
            G_ODR_6_5Hz = 0x0B,
        }lsm6dso_g_odr_t;

};
#endif