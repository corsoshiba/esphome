from typing import Any

import esphome.codegen as cg
import esphome.config_validation as cv

from . import generate, schema

CONF_min_value = "min_value"
CONF_max_value = "max_value"
CONF_auto_min_value = "auto_min_value"
CONF_auto_max_value = "auto_max_value"
CONF_step = "step"

OpenthermInput = generate.opentherm_ns.class_("OpenthermInput")


def validate_min_value_less_than_max_value(conf):
    if (
        CONF_min_value in conf
        and CONF_max_value in conf
        and conf[CONF_min_value] > conf[CONF_max_value]
    ):
        raise cv.Invalid(f"{CONF_min_value} must be less than {CONF_max_value}")
    return conf


def input_schema(entity: schema.InputSchema) -> cv.Schema:
    result = cv.Schema(
        {
            cv.Optional(CONF_min_value, entity.range[0]): cv.float_range(
                entity.range[0], entity.range[1]
            ),
            cv.Optional(CONF_max_value, entity.range[1]): cv.float_range(
                entity.range[0], entity.range[1]
            ),
        }
    )
    result = result.add_extra(validate_min_value_less_than_max_value)
    result = result.extend({cv.Optional(CONF_step, False): cv.float_})
    if entity.auto_min_value is not None:
        result = result.extend({cv.Optional(CONF_auto_min_value, False): cv.boolean})
    if entity.auto_max_value is not None:
        result = result.extend({cv.Optional(CONF_auto_max_value, False): cv.boolean})

    return result


def generate_setters(entity: cg.MockObj, conf: dict[str, Any]) -> None:
    generate.add_property_set(entity, CONF_min_value, conf)
    generate.add_property_set(entity, CONF_max_value, conf)
    generate.add_property_set(entity, CONF_auto_min_value, conf)
    generate.add_property_set(entity, CONF_auto_max_value, conf)
