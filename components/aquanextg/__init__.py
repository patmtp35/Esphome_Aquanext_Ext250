import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PROBLEM,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
    ICON_THERMOMETER,
)

# Dépendances ESPHome
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

# Namespace C++
aquanext_ns = cg.esphome_ns.namespace("aquanextg")
AquaNextComponent = aquanext_ns.class_(
    "AquaNextComponent", cg.Component, uart.UARTDevice
)

# Clés de configuration optionnelles pour chaque sensor
CONF_TEMPERATURE_TARGET  = "temperature_target"
CONF_TEMPERATURE_DOME    = "temperature_dome"
CONF_TEMPERATURE_AIR     = "temperature_air"
CONF_TEMPERATURE_EVAP    = "temperature_evap"
CONF_TEMPERATURE_TW1     = "temperature_tw1"
CONF_TEMPERATURE_TW2     = "temperature_tw2"
CONF_TEMPERATURE_TW3     = "temperature_tw3"
CONF_TEMPERATURE_T_HP    = "temperature_t_hp"
CONF_TEMPERATURE_T_MAX   = "temperature_t_max"
CONF_TEMPERATURE_T_MIN   = "temperature_t_min"
CONF_HP_HOURS            = "hp_hours"
CONF_HE_HOURS            = "he_hours"
CONF_MODE                = "mode"
CONF_FW_VERSION          = "fw_version"
CONF_POWER               = "power"
CONF_HEAT_PUMP_ACTIVE    = "heat_pump_active"
CONF_HEAT_ELEMENT_ACTIVE = "heat_element_active"
CONF_ERROR_PRESENT       = "error_present"
CONF_SETTING_ANTIBACT    = "setting_antibact"
CONF_SETTING_GREEN       = "setting_green"
CONF_SETTING_VOYAGE      = "setting_voyage"

# Schéma de validation YAML
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AquaNextComponent),
            # Sensors température (tous optionnels)
            cv.Optional(CONF_TEMPERATURE_TARGET): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_DOME): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_AIR): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_EVAP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_TW1): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_TW2): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_TW3): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_T_HP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_T_MAX): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
            ),
            cv.Optional(CONF_TEMPERATURE_T_MIN): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
            ),
            cv.Optional(CONF_HP_HOURS): sensor.sensor_schema(
                unit_of_measurement=UNIT_HOUR,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                icon="mdi:heat-pump",
            ),
            cv.Optional(CONF_HE_HOURS): sensor.sensor_schema(
                unit_of_measurement=UNIT_HOUR,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                icon="mdi:heating-coil",
            ),
            # Text sensors
            cv.Optional(CONF_MODE): text_sensor.text_sensor_schema(
                icon="mdi:water-boiler",
            ),
            cv.Optional(CONF_FW_VERSION): text_sensor.text_sensor_schema(
                icon="mdi:chip",
            ),
            # Binary sensors
            cv.Optional(CONF_POWER): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_POWER,
            ),
            cv.Optional(CONF_HEAT_PUMP_ACTIVE): binary_sensor.binary_sensor_schema(
                icon="mdi:heat-pump",
            ),
            cv.Optional(CONF_HEAT_ELEMENT_ACTIVE): binary_sensor.binary_sensor_schema(
                icon="mdi:heating-coil",
            ),
            cv.Optional(CONF_ERROR_PRESENT): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
            ),
            cv.Optional(CONF_SETTING_ANTIBACT): binary_sensor.binary_sensor_schema(
                icon="mdi:bacteria-outline",
            ),
            cv.Optional(CONF_SETTING_GREEN): binary_sensor.binary_sensor_schema(
                icon="mdi:leaf",
            ),
            cv.Optional(CONF_SETTING_VOYAGE): binary_sensor.binary_sensor_schema(
                icon="mdi:airplane",
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Enregistrer chaque sensor si présent dans le YAML
    sensor_map = {
        CONF_TEMPERATURE_TARGET:  "set_temperature_target_sensor",
        CONF_TEMPERATURE_DOME:    "set_temperature_dome_sensor",
        CONF_TEMPERATURE_AIR:     "set_temperature_air_sensor",
        CONF_TEMPERATURE_EVAP:    "set_temperature_evap_sensor",
        CONF_TEMPERATURE_TW1:     "set_temperature_tw1_sensor",
        CONF_TEMPERATURE_TW2:     "set_temperature_tw2_sensor",
        CONF_TEMPERATURE_TW3:     "set_temperature_tw3_sensor",
        CONF_TEMPERATURE_T_HP:    "set_temperature_t_hp_sensor",
        CONF_TEMPERATURE_T_MAX:   "set_temperature_t_max_sensor",
        CONF_TEMPERATURE_T_MIN:   "set_temperature_t_min_sensor",
        CONF_HP_HOURS:            "set_hp_hours_sensor",
        CONF_HE_HOURS:            "set_he_hours_sensor",
    }
    for conf_key, setter in sensor_map.items():
        if conf_key in config:
            sens = await sensor.new_sensor(config[conf_key])
            cg.add(getattr(var, setter)(sens))

    text_sensor_map = {
        CONF_MODE:       "set_mode_text_sensor",
        CONF_FW_VERSION: "set_fw_version_text_sensor",
    }
    for conf_key, setter in text_sensor_map.items():
        if conf_key in config:
            sens = await text_sensor.new_text_sensor(config[conf_key])
            cg.add(getattr(var, setter)(sens))

    binary_sensor_map = {
        CONF_POWER:               "set_power_binary_sensor",
        CONF_HEAT_PUMP_ACTIVE:    "set_heat_pump_active_binary_sensor",
        CONF_HEAT_ELEMENT_ACTIVE: "set_heat_element_active_binary_sensor",
        CONF_ERROR_PRESENT:       "set_error_present_binary_sensor",
        CONF_SETTING_ANTIBACT:    "set_setting_antibact_binary_sensor",
        CONF_SETTING_GREEN:       "set_setting_green_binary_sensor",
        CONF_SETTING_VOYAGE:      "set_setting_voyage_binary_sensor",
    }
    for conf_key, setter in binary_sensor_map.items():
        if conf_key in config:
            sens = await binary_sensor.new_binary_sensor(config[conf_key])
            cg.add(getattr(var, setter)(sens))
