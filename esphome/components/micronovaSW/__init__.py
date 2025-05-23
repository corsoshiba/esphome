from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
)

from esphome.core import TimePeriod

CODEOWNERS = ["@jorre05"]

DEPENDENCIES = ["uart"]

CONF_MICRONOVASW_ID = "micronovaSW_id"
CONF_ENABLE_RX_PIN = "enable_rx_pin"
CONF_SERIAL_REPLY_DELAY = "serial_reply_delay"
CONF_MEMORY_LOCATION = "memory_location"
CONF_MEMORY_ADDRESS = "memory_address"


micronovaSW_ns = cg.esphome_ns.namespace("micronovaSW")

MicroNovaSWFunctions = micronovaSW_ns.enum("MicroNovaSWFunctions", is_class=True)
MICRONOVA_FUNCTIONS_ENUM = {
    "STOVE_FUNCTION_SWITCH": MicroNovaSWFunctions.STOVE_FUNCTION_SWITCH,
    "STOVE_FUNCTION_ROOM_TEMPERATURE": MicroNovaSWFunctions.STOVE_FUNCTION_ROOM_TEMPERATURE,
    "STOVE_FUNCTION_THERMOSTAT_TEMPERATURE": MicroNovaSWFunctions.STOVE_FUNCTION_THERMOSTAT_TEMPERATURE,
    "STOVE_FUNCTION_FUMES_TEMPERATURE": MicroNovaSWFunctions.STOVE_FUNCTION_FUMES_TEMPERATURE,
    "STOVE_FUNCTION_STOVE_POWER": MicroNovaSWFunctions.STOVE_FUNCTION_STOVE_POWER,
    "STOVE_FUNCTION_FAN_SPEED": MicroNovaSWFunctions.STOVE_FUNCTION_FAN_SPEED,
    "STOVE_FUNCTION_STOVE_STATE": MicroNovaSWFunctions.STOVE_FUNCTION_STOVE_STATE,
    "STOVE_FUNCTION_MEMORY_ADDRESS_SENSOR": MicroNovaSWFunctions.STOVE_FUNCTION_MEMORY_ADDRESS_SENSOR,
    "STOVE_FUNCTION_WATER_TEMPERATURE": MicroNovaSWFunctions.STOVE_FUNCTION_WATER_TEMPERATURE,
    "STOVE_FUNCTION_WATER_PRESSURE": MicroNovaSWFunctions.STOVE_FUNCTION_WATER_PRESSURE,
    "STOVE_FUNCTION_POWER_LEVEL": MicroNovaSWFunctions.STOVE_FUNCTION_POWER_LEVEL,
    "STOVE_FUNCTION_CUSTOM": MicroNovaSWFunctions.STOVE_FUNCTION_CUSTOM,
}

MicroNovaSW = micronovaSW_ns.class_("MicroNovaSW", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MicroNovaSW),
            cv.Required(CONF_ENABLE_RX_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SERIAL_REPLY_DELAY, default="60ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=60),max=TimePeriod(milliseconds=65535)),
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("60s"))
)


def MICRONOVA_LISTENER_SCHEMA(default_memory_location, default_memory_address):
    return cv.Schema(
        {
            cv.GenerateID(CONF_MICRONOVASW_ID): cv.use_id(MicroNovaSW),
            cv.Optional(
                CONF_MEMORY_LOCATION, default=default_memory_location
            ): cv.hex_int_range(),
            cv.Optional(
                CONF_MEMORY_ADDRESS, default=default_memory_address
            ): cv.hex_int_range(),
        }
    )


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    enable_rx_pin = await cg.gpio_pin_expression(config[CONF_ENABLE_RX_PIN])
    cg.add(var.set_enable_rx_pin(enable_rx_pin))
    cg.add(var.set_serial_reply_delay(config[CONF_SERIAL_REPLY_DELAY].total_milliseconds))