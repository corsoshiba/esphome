from esphome import automation
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import modbus, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_ENERGY,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_POWER,
    CONF_POWER_FACTOR,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    ICON_CURRENT_AC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

AUTO_LOAD = ["modbus"]

pzemac_ns = cg.esphome_ns.namespace("pzemac")
PZEMAC = pzemac_ns.class_("PZEMAC", cg.PollingComponent, modbus.ModbusDevice)

# Actions
ResetEnergyAction = pzemac_ns.class_("ResetEnergyAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PZEMAC),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_POWER_FACTOR,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(modbus.modbus_device_schema(0x01))
)


@automation.register_action(
    "pzemac.reset_energy",
    ResetEnergyAction,
    maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PZEMAC),
        }
    ),
)
async def reset_energy_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await modbus.register_modbus_device(var, config)

    if CONF_VOLTAGE in config:
        conf = config[CONF_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_voltage_sensor(sens))
    if CONF_CURRENT in config:
        conf = config[CONF_CURRENT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_current_sensor(sens))
    if CONF_POWER in config:
        conf = config[CONF_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_power_sensor(sens))
    if CONF_ENERGY in config:
        conf = config[CONF_ENERGY]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_energy_sensor(sens))
    if CONF_FREQUENCY in config:
        conf = config[CONF_FREQUENCY]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_frequency_sensor(sens))
    if CONF_POWER_FACTOR in config:
        conf = config[CONF_POWER_FACTOR]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_power_factor_sensor(sens))
