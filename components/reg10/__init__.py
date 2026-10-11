"""Read-only REG10 UART client and optional cached Modbus TCP server."""
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor, socket, uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart", "esp32"]
AUTO_LOAD = ["sensor", "binary_sensor", "network", "socket"]
MULTI_CONF = False
reg10_ns = cg.esphome_ns.namespace("reg10")
REG10 = reg10_ns.class_("REG10", cg.PollingComponent, uart.UARTDevice)
TEMP_KEYS = [f"temperature_{i}" for i in range(1, 5)]
RAW_KEYS = ["flow_1_raw", "flow_2_raw", "outputs_raw"]
DIAG_KEYS = ["valid_mask", "temperature_valid_mask", "good_frames", "crc_errors", "timeouts"]
AGE_KEYS = ["temperature_age", "output_age", "flow_age"]
BINARY_KEYS = ["all_online", "temperature_online", "output_online", "flow_online"]
OUTPUT_KEYS = [f"out{i}" for i in range(1, 6)]
schema = {
    cv.GenerateID(): cv.declare_id(REG10),
    cv.Required("direction_pin"): pins.internal_gpio_output_pin_schema,
    cv.Optional("slave_address", default=1): cv.int_range(min=1, max=127),
    cv.Optional("master_address", default=128): cv.int_range(min=128, max=255),
    cv.Optional("response_timeout", default="1s"): cv.positive_time_period_milliseconds,
    cv.Optional("request_gap", default="100ms"): cv.positive_time_period_milliseconds,
    cv.Optional("stale_after", default="60s"): cv.positive_time_period_milliseconds,
    cv.Optional("retries", default=2): cv.int_range(min=0, max=5),
    cv.Optional("tcp_port", default=502): cv.int_range(min=0, max=65535),
    cv.Optional("tcp_unit_id", default=1): cv.int_range(min=1, max=247),
}
for key in TEMP_KEYS:
    schema[cv.Optional(key)] = sensor.sensor_schema(
        unit_of_measurement="°C", accuracy_decimals=1,
        device_class="temperature", state_class="measurement")
for key in RAW_KEYS:
    schema[cv.Optional(key)] = sensor.sensor_schema(accuracy_decimals=0)
for key in DIAG_KEYS:
    schema[cv.Optional(key)] = sensor.sensor_schema(accuracy_decimals=0, entity_category="diagnostic")
for key in AGE_KEYS:
    schema[cv.Optional(key)] = sensor.sensor_schema(
        unit_of_measurement="s", accuracy_decimals=0, entity_category="diagnostic")
for key in BINARY_KEYS:
    schema[cv.Optional(key)] = binary_sensor.binary_sensor_schema(
        device_class="connectivity", entity_category="diagnostic")
for key in OUTPUT_KEYS:
    schema[cv.Optional(key)] = binary_sensor.binary_sensor_schema()


def validate_config(config):
    timeout = config["response_timeout"].total_milliseconds
    gap = config["request_gap"].total_milliseconds
    stale = config["stale_after"].total_milliseconds
    interval = config["update_interval"].total_milliseconds
    if not 100 <= timeout <= 10000:
        raise cv.Invalid("response_timeout must be between 100ms and 10s")
    if not 20 <= gap <= 2000:
        raise cv.Invalid("request_gap must be between 20ms and 2s")
    if not 2000 <= interval <= 600000:
        raise cv.Invalid("update_interval must be between 2s and 10min")
    if not interval < stale <= 3600000:
        raise cv.Invalid("stale_after must exceed update_interval and be <= 1h")
    if config["tcp_port"] and hasattr(socket, "consume_sockets"):
        if hasattr(socket, "SocketType") and hasattr(socket.SocketType, "TCP_LISTEN"):
            config = socket.consume_sockets(2, "reg10")(config)
            config = socket.consume_sockets(1, "reg10", socket.SocketType.TCP_LISTEN)(config)
        else:
            config = socket.consume_sockets(3, "reg10")(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(schema).extend(cv.polling_component_schema("15s")).extend(uart.UART_DEVICE_SCHEMA),
    validate_config)
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "reg10", require_tx=True, require_rx=True, data_bits=8, parity="NONE", stop_bits=1)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    direction_pin = await cg.gpio_pin_expression(config["direction_pin"])
    cg.add(var.set_direction_pin(direction_pin))
    for key in ["slave_address", "master_address", "retries", "tcp_port", "tcp_unit_id"]:
        cg.add(getattr(var, f"set_{key}")(config[key]))
    for key in ["response_timeout", "request_gap", "stale_after"]:
        cg.add(getattr(var, f"set_{key}")(config[key].total_milliseconds))
    for i, key in enumerate(TEMP_KEYS):
        if key in config:
            cg.add(var.set_temperature_sensor(i, await sensor.new_sensor(config[key])))
    for i, key in enumerate(["flow_1_raw", "flow_2_raw"]):
        if key in config:
            cg.add(var.set_flow_sensor(i, await sensor.new_sensor(config[key])))
    for key in ["outputs_raw"] + DIAG_KEYS + AGE_KEYS:
        if key in config:
            cg.add(getattr(var, f"set_{key}_sensor")(await sensor.new_sensor(config[key])))
    for i, key in enumerate(OUTPUT_KEYS):
        if key in config:
            cg.add(var.set_output_sensor(i, await binary_sensor.new_binary_sensor(config[key])))
    for key in BINARY_KEYS:
        if key in config:
            cg.add(getattr(var, f"set_{key}_sensor")(
                await binary_sensor.new_binary_sensor(config[key])))
