import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from . import AqaraCurtainComponent, aqara_curtain_ns

DEPENDENCIES = ["aqara_curtain_component"]

AqaraCurtainCover = aqara_curtain_ns.class_("AqaraCurtainCover", cover.Cover)

CONF_AQARA_CURTAIN_ID = "aqara_curtain_id"

CONFIG_SCHEMA = cover.cover_schema(AqaraCurtainCover).extend(
    {
        cv.Required(CONF_AQARA_CURTAIN_ID): cv.use_id(AqaraCurtainComponent),
    }
)


async def to_code(config):
    var = await cover.new_cover(config)

    parent = await cg.get_variable(config[CONF_AQARA_CURTAIN_ID])
    cg.add(parent.set_cover(var))
