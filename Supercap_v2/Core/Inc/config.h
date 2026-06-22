#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Power Limit ---------------- */
#define POWER_LIMIT 45 // in W, this should be read from a CAN message
#define ref_system_indicates_off 1 //should be removed once CAN is setup

/* ---------------- Safety ---------------- */
#define MINIMUM_CAPACITOR_ENERGY 450.0f // The energy value the capacitor should never go under (Joules)
#define MAX_CHARGE_CURRENT 5.0     // max allowed charge current
#define MAX_DISCHARGE_CURRENT 5.0 // max allowed discharge current
#define MAX_DUTY_CYCLE 0.95f // maximum duty cycle of pwm
#define MIN_DUTY_CYCLE 0.05f // minimum duty cycle of pwm

/* ---------------- Capacitor Parameters ---------------- */
#define CAPACITOR_VOLTAGE_MAX 27  // The voltage rating of the capacitor bank
#define CAPACITANCE_TOTAL 5 // The total capacitance of the capacitor bank in Farads
#define SYSTEM_R 0.085f  //resistance of RDS_on of High side MOSFET + RDS_on low side MOSFET + Inductor DCR

/* ---------------- Loop timing ---------------- */
#define CONTROL_DELAY_MS             2U

/* ---------------- ADC scaling ---------------- */
#define BUS_SENSE_GAIN               11.0f // gain to undo the voltage divider drop
#define CAP_SENSE_GAIN               11.0f // gain to undo the voltage divider drop
#define ADC_ZERO_CLAMP_V             0.03f //the noise threshold, below this value means assume voltage == 0

/* ---------------- INA240 conversion to current ---------------- */
#define CURRENT_SHUNT_SCALE			10.0f // conversion from voltage to current: 1 / (0.002 ohms * 50x gain) = 10.0f

/* ---------------- PD Controller ---------------- */
#define KP_GAIN 0.01f
#define KI_GAIN 0.001f
#define MAX_INTEGRAL_TERM 0.05f // The max the integral term contributes to the duty cycle

/* ---------------- Filters ---------------- */
#define IBAT_DEADBAND_AMPS 0.05f //currents below this value are just converted to 0 for readability
#define ICAP_DEADBAND_AMPS 0.05f //currents below this value are just converted to 0 for readability
#define IMOTOR_DEADBAND_AMPS 0.05f //currents below this value are just converted to 0 for readability
#define FILTER_ALPHA 0.05f //how much to use current value compared to old value in filtering


#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H__ */
