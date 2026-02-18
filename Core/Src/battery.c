#include "battery.h"
#include "bq27427_reg.h"
#include "main.h"

// Cached battery state
typedef struct {
    uint16_t voltage_mV;
    int16_t current_mA;
    uint16_t soc_percent;
    bool is_charging;
    bool is_full;
    bool is_low;
    bool is_critical;
    uint32_t last_update;
} BatteryState_t;

static BatteryState_t battery_state = {0};

static uint8_t BATTERY_EstimateSOC_FromVoltage(uint16_t voltage_mV);

/**
 * @brief Initialize the BQ27427 fuel gauge
 * @return true if initialization successful, false otherwise
 */
bool BATTERY_Init(void)
{
    if (!bq27427_init()) {
        return false;
    }

    uint16_t device_type = bq27427_device_type();
    if (device_type != 0x0427) {
        return false;
    }

    // Set chemistry profile to CHEM_B (4.2V LiPo) if not already set
    bq27427_chemistry_t current_chem = bq27427_chem_id();
    if (current_chem != BQ27427_CHEM_B) {
        if (!bq27427_set_chem_id(BQ27427_CHEM_B)) {
            return false;
        }
        HAL_Delay(100);
    }

    // Check if already configured correctly
    uint16_t current_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    uint16_t current_terminate_voltage = bq27427_terminate_voltage();
    uint16_t current_taper_rate = bq27427_taper_rate();
    uint16_t current_taper_voltage = bq27427_taper_voltage();

    bool needs_config = (current_capacity != 4900) ||
                        (current_terminate_voltage != 3000) ||
                        (current_taper_rate != 100) ||
                        (current_taper_voltage != 4040);

    if (needs_config) {
        if (!bq27427_enter_config(true)) {
            return false;
        }

        bq27427_set_current_polarity(0); // 0 = Positive current means battery is charging
        bq27427_set_capacity(4900);
        bq27427_set_terminate_voltage(3000);
        bq27427_set_taper_voltage(4040);
        bq27427_set_taper_rate(100);  // (300mAh / 30mA) * 10 = 100, CUTS OFF AT 26mA CHARGING

        if (!bq27427_exit_config(true)) {
            return false;
        }

        HAL_Delay(500);
    }

    battery_state.last_update = 0;
    return true;
}

/**
 * @brief Update all battery parameters (rate-limited to 500ms)
 * @return true if update successful
 */
bool BATTERY_UpdateState(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - battery_state.last_update) < 500) {
        return true;
    }

    battery_state.voltage_mV = bq27427_voltage();
    battery_state.current_mA = bq27427_current(BQ27427_CURRENT_AVG);
    battery_state.soc_percent = bq27427_soc(BQ27427_SOC_FILTERED);
    battery_state.is_charging = bq27427_chg_flag();
    battery_state.is_full = bq27427_fc_flag();
    battery_state.is_low = bq27427_soc_flag();
    battery_state.is_critical = bq27427_socf_flag();

    // If gauge reports 0% but voltage is good, use estimation
    if (battery_state.soc_percent == 0 && battery_state.voltage_mV > 3200) {
        battery_state.soc_percent = BATTERY_EstimateSOC_FromVoltage(battery_state.voltage_mV);
    }

    battery_state.last_update = now;
    return true;
}

uint16_t BATTERY_GetVoltage(void)   { return battery_state.voltage_mV; }
int16_t  BATTERY_GetCurrent(void)   { return battery_state.current_mA; }
uint16_t BATTERY_GetSOC(void)       { return battery_state.soc_percent; }
bool     BATTERY_IsCharging(void)   { return battery_state.is_charging; }
bool     BATTERY_IsFull(void)       { return battery_state.is_full; }
bool     BATTERY_IsLow(void)        { return battery_state.is_low; }
bool     BATTERY_IsCritical(void)   { return battery_state.is_critical; }

/**
 * @brief Estimate SOC percentage from battery voltage (LiPo curve)
 * @param voltage_mV Battery voltage in millivolts
 * @return Estimated SOC in percent (0-100)
 */
static uint8_t BATTERY_EstimateSOC_FromVoltage(uint16_t voltage_mV)
{
    if (voltage_mV >= 4200) return 100;
    if (voltage_mV >= 4100) return 90;
    if (voltage_mV >= 4000) return 80;
    if (voltage_mV >= 3950) return 75;
    if (voltage_mV >= 3900) return 70;
    if (voltage_mV >= 3850) return 65;
    if (voltage_mV >= 3800) return 60;
    if (voltage_mV >= 3750) return 55;
    if (voltage_mV >= 3700) return 50;
    if (voltage_mV >= 3650) return 40;
    if (voltage_mV >= 3600) return 30;
    if (voltage_mV >= 3500) return 20;
    if (voltage_mV >= 3400) return 10;
    if (voltage_mV >= 3300) return 5;
    if (voltage_mV >= 3200) return 2;
    return 1;
}
