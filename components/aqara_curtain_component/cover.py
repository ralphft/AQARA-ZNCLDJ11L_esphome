import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import CONF_ID
from . import AqaraCurtainComponent, aqara_curtain_ns

DEPENDENCIES = ["aqara_curtain_component"]

AqaraCurtainCover = aqara_curtain_ns.class_("AqaraCurtainCover", cover.Cover)

CONF_AQARA_CURTAIN_ID = "aqara_curtain_id"

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(AqaraCurtainCover),
        cv.Required(CONF_AQARA_CURTAIN_ID): cv.use_id(AqaraCurtainComponent),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cover.register_cover(var, config)

    parent = await cg.get_variable(config[CONF_AQARA_CURTAIN_ID])
    cg.add(parent.set_cover(var))


