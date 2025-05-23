import esphome.codegen as cg
from esphome.components import cover
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_SOURCE_ID,
)
from esphome.core.entity_helpers import inherit_property_from

from .. import copy_ns

CopyCover = copy_ns.class_("CopyCover", cover.Cover, cg.Component)


CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(CopyCover),
        cv.Required(CONF_SOURCE_ID): cv.use_id(cover.Cover),
    }
).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = cv.All(
    inherit_property_from(CONF_ICON, CONF_SOURCE_ID),
    inherit_property_from(CONF_ENTITY_CATEGORY, CONF_SOURCE_ID),
    inherit_property_from(CONF_DEVICE_CLASS, CONF_SOURCE_ID),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cover.register_cover(var, config)
    await cg.register_component(var, config)

    source = await cg.get_variable(config[CONF_SOURCE_ID])
    cg.add(var.set_source(source))
