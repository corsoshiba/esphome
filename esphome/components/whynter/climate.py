import esphome.codegen as cg
from esphome.components import climate_ir
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_USE_FAHRENHEIT

AUTO_LOAD = ["climate_ir"]

whynter_ns = cg.esphome_ns.namespace("whynter")
Whynter = whynter_ns.class_("Whynter", climate_ir.ClimateIR)


CONFIG_SCHEMA = climate_ir.CLIMATE_IR_WITH_RECEIVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(Whynter),
        cv.Optional(CONF_USE_FAHRENHEIT, default=False): cv.boolean,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate_ir.register_climate_ir(var, config)
    cg.add(var.set_fahrenheit(config[CONF_USE_FAHRENHEIT]))
