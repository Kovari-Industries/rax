"""Shared Evo request schemas and response checks. No transport or robot access.

The native tool JSON is copied byte-for-byte from Evo's checked-in contract.
The real backend and simulation adapters use this same validator.
"""
import json
import math
from pathlib import Path

TOOLS = json.loads(Path(__file__).with_name("tools.json").read_text())
ELEVATOR_TOOLS = json.loads(Path(__file__).with_name("elevator_tools.json").read_text())
CLIENT_TOOLS = TOOLS + ELEVATOR_TOOLS
BY_NAME = {tool["name"]: tool for tool in CLIENT_TOOLS}
MOTION = frozenset(
    "evo_" + name
    for name in (
        "move_joints",
        "move_eef",
        "gripper",
        "move_carriage",
        "move_neck",
        "drive",
        "move_body",
        "follow_path",
        "move_pair",
        "set_grip",
    )
)
def validate_value(value, schema):
    """Validate the restricted, checked-in tool schema without a runtime dependency."""
    if "oneOf" in schema:
        matches = 0
        for choice in schema["oneOf"]:
            try:
                validate_value(value, choice)
                matches += 1
            except ValueError:
                pass
        if matches != 1:
            raise ValueError("arguments must match exactly one action schema")
        return
    if "const" in schema and value != schema["const"]:
        raise ValueError("invalid action kind")
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError("value is not an allowed choice")
    kind = schema.get("type")
    if kind == "object":
        if not isinstance(value, dict):
            raise ValueError("expected an object")
        properties = schema.get("properties", {})
        if set(schema.get("required", ())) - value.keys():
            raise ValueError("missing required arguments")
        extra = schema.get("additionalProperties", True)
        if extra is False and value.keys() - properties.keys():
            raise ValueError("unknown arguments")
        if (
            not schema.get("minProperties", 0)
            <= len(value)
            <= schema.get("maxProperties", 128)
        ):
            raise ValueError("invalid object size")
        for key, item in value.items():
            if not isinstance(key, str) or len(key) > 128:
                raise ValueError("invalid field name")
            validate_value(
                item, properties.get(key, extra if isinstance(extra, dict) else {})
            )
    elif kind == "array":
        if not isinstance(value, (list, tuple)) or not schema.get("minItems", 0) <= len(
            value
        ) <= schema.get("maxItems", 128):
            raise ValueError("invalid array size")
        for item in value:
            validate_value(item, schema["items"])
    elif kind in ("number", "integer"):
        if kind == "integer" and (
            not isinstance(value, int) or isinstance(value, bool)
        ):
            raise ValueError("expected an integer")
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError("expected a finite number")
        if "minimum" in schema and value < schema["minimum"]:
            raise ValueError("number is below minimum")
        if "maximum" in schema and value > schema["maximum"]:
            raise ValueError("number exceeds maximum")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            raise ValueError("number is below minimum")
    elif kind == "boolean":
        if not isinstance(value, bool):
            raise ValueError("expected a boolean")
    elif kind == "string":
        if not isinstance(value, str) or not schema.get("minLength", 1) <= len(
            value
        ) <= schema.get("maxLength", 1024):
            raise ValueError("expected a nonempty bounded string")



def validate_request(name, arguments):
    if name not in BY_NAME or not isinstance(arguments, dict):
        raise ValueError("unknown Evo tool or arguments")
    validate_value(arguments, BY_NAME[name]["inputSchema"])


def validate_acceptance(value):
    if (not isinstance(value, dict) or value.get("state") != "accepted"
            or not isinstance(value.get("action_id"), str) or not value["action_id"]):
        raise ValueError("invalid Evo action acceptance")


def validate_terminal(value):
    if (not isinstance(value, dict) or value.get("state") not in ("succeeded", "cancelled", "failed")
            or type(value.get("physical_stop_verified")) is not bool):
        raise ValueError("invalid Evo terminal action result")
