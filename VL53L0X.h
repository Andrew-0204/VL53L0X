#ifndef VL53L0X_h
#define VL53L0X_h

#include <Arduino.h>
#include <Wire.h>

class VL53L0X
{
  public:
    // register addresses from API vl53l0x_device.h (ordered as listed there)
    enum regAddr
    {
      SYSRANGE_START                              = 0x00, //thanh ghi bat dau

      SYSTEM_THRESH_HIGH                          = 0x0C, //thanh ghi nguong cao
      SYSTEM_THRESH_LOW                           = 0x0E, //thanh ghi nguong thap

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,//thanh ghi ket noi thiet lap cau hinh
      
      SYSTEM_RANGE_CONFIG                         = 0x09,//thanh ghi pham vi thiet lap
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,//thanh ghi cac giai doan do luong

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,//thanh ghi ngat cac cong thiet lap cau hinh

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,//thanh ghi cong

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,//thanh ghi loai bo cac thiet lap cu

      RESULT_INTERRUPT_STATUS                     = 0x13,//thanh ghi trang thai ngat
      RESULT_RANGE_STATUS                         = 0x14,//thanh ghi nguong trang thai

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,//thanh ghi chuyen tiep moi truong loi cua window
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,//thanh ghi chuyen tiep gioi han so loi
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,//thanh ghi gioi thieu moi truong loi cua window
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,//thanh ghi gioi thieu gioi han so loi
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,//thanh ghi danh gia tin hieu dinh

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,//thanh ghi gioi han phan bu thuat toan 

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,//thanh ghi dia chi slave I2c

      MSRC_CONFIG_CONTROL                         = 0x60,//thanh ghi dieu khien thiet lap chia se tai nguyen

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,//thanh ghi thiet lap gioi han nho nhat cua ty so tin hieu nhieu
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,//thanh ghi thiet lap dai gia tri cua giai doan xung thap
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,//thanh ghi thiet lap dai gia tri cua giai doan xung cao
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,//thanh ghi chuyen tiep gia tri tinh toan gioi han nho nhat

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,//thanh ghi thiet lap gioi han nho nhat cua ty so tin hieu nhieu
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,//thanh ghi thiet lap dai gia tri cua giai doan xung thap
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,//thanh ghi thiet lap dai gia tri cua giai doan xung cao
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,//thanh ghi chuyen tiep gia tri tinh toan gioi han nho nhat

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,//thanh ghi dai thiet lap xung nhip cao
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,//thanh ghi dai thiet lap xung nhip thap

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,//thanh ghi dai thiet lap cac giai doan cua laser
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,//thanh ghi dai thiet lap thoi gian cho cua giai doan xung cao
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,//thanh ghi dai thiet lap thoi gian cho cua giai doan xung thap

      SYSTEM_HISTOGRAM_BIN                        = 0x81,//thanh ghi he thong bieu do tin hieu
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,//thanh ghi thiet lap lua chon diem bat dau cua bieu do
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,//thanh ghi thiet lap bieu do dieu khien gia tri doc ra

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,//thanh ghi dai thiet lap cac giai doan cua laser
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,//thanh ghi dai thiet lap thoi gian cho cua giai doan xung cao
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,//thanh ghi dai thiet lap thoi gian cho cua giat doan xung thap
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,//thanh ghi ti le bu nhieu

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,//thanh ghi thiet lap thoi gian cho cua cac gian doan chia se tai nguyen

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,//thanh ghi reset phan mem
      IDENTIFICATION_MODEL_ID                     = 0xC0,//thanh ghi nhan biet ID cua model
      IDENTIFICATION_REVISION_ID                  = 0xC2,//thanh ghi nhan biet ID cua cac sua doi

      OSC_CALIBRATE_VAL                           = 0xF8,//thanh ghi hieu chinh gia tri dao dong

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,//thanh ghi thiet lap chung do rong cua dai laser
      //cac thanh ghi duong truyen tin hieu chung duoc 
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,//thanh ghi lua chon duong truyen
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,//thanh ghi so luong duong truyen duoc yeu cau
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,//thanh ghi duong truyen duoc kich hoat thay the
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,//thanh ghi ve dien ap

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,//thanh ghi tep thong tin ho tro SDA,SCL

      ALGO_PHASECAL_LIM                           = 0x30,//thanh ghi gioi han cua thuat toan
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,//thanh ghi thiet lap tinh toan thoi gian cho
    };

    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

    uint8_t last_status; // status of last I2C transmission

    VL53L0X();

    void setBus(TwoWire * bus) { this->bus = bus; }
    TwoWire * getBus() { return bus; }

    void setAddress(uint8_t new_addr);
    inline uint8_t getAddress() { return address; }

    bool init(bool io_2v8 = true);

    void writeReg(uint8_t reg, uint8_t value);
    void writeReg16Bit(uint8_t reg, uint16_t value);
    void writeReg32Bit(uint8_t reg, uint32_t value);
    uint8_t readReg(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);
    uint32_t readReg32Bit(uint8_t reg);

    void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
    void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

    bool setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit();

    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod(vcselPeriodType type);

    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();
    uint16_t readRangeContinuousMillimeters();
    uint16_t readRangeSingleMillimeters();

    inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    inline uint16_t getTimeout() { return io_timeout; }
    bool timeoutOccurred();

  private:
    // TCC: Target CentreCheck
    // MSRC: Minimum Signal Rate Check
    // DSS: Dynamic Spad Selection

    struct SequenceStepEnables
    {
      boolean tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    TwoWire * bus;
    uint8_t address;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;

    bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

    void getSequenceStepEnables(SequenceStepEnables * enables);
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};

#endif



