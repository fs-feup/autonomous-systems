#!/usr/bin/env python3

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

OUTPUT_TYPE = "custom_interfaces/msg/ControlCommand"
LEGACY_INPUT_TYPE = "custom_interfaces/msg/PreviousControlCommand"

# Edit these values for no-args usage:
DEFAULT_INPUT_BAG = "FSG\ Autocross\ DV"
DEFAULT_OUTPUT_BAG = "control_command_converted"
DEFAULT_INPUT_TOPIC = "/as_msgs/controls"
DEFAULT_OUTPUT_TOPIC = "/control/command"
DEFAULT_INPUT_STORAGE_ID = "mcap"
DEFAULT_OUTPUT_STORAGE_ID = "mcap"


def _build_control_from_legacy(legacy_msg):
    control_cls = get_message(OUTPUT_TYPE)
    out = control_cls()
    out.header = legacy_msg.header
    out.throttle_fl = 0.0
    out.throttle_fr = 0.0
    out.throttle_rl = legacy_msg.throttle
    out.throttle_rr = legacy_msg.throttle
    out.steering = legacy_msg.steering
    return out


def _deserialize_control_message(
    serialized_message, input_type, output_msg_cls, legacy_msg_cls
):
    if input_type == LEGACY_INPUT_TYPE:
        legacy_msg = deserialize_message(serialized_message, legacy_msg_cls)
        return _build_control_from_legacy(legacy_msg), False, False

    try:
        return deserialize_message(serialized_message, output_msg_cls), False, False
    except Exception:
        try:
            legacy_msg = deserialize_message(serialized_message, legacy_msg_cls)
            return _build_control_from_legacy(legacy_msg), True, False
        except Exception:
            return None, False, True


def convert_bag(
    input_bag_path: str,
    output_bag_path: str,
    input_topic: str,
    output_topic: str,
    input_storage_id: str,
    output_storage_id: str,
) -> int:
    input_path = Path(input_bag_path)
    if not input_path.exists():
        raise FileNotFoundError(f"Input bag path does not exist: {input_bag_path}")

    output_path = Path(output_bag_path)
    if output_path.exists():
        raise FileExistsError(
            f"Output bag path already exists: {output_bag_path}. "
            "Please choose a new path or remove the previous output."
        )

    reader = rosbag2_py.SequentialReader()
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag_path, storage_id=input_storage_id),
        converter_options,
    )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if input_topic not in topic_types:
        available_topics = ", ".join(sorted(topic_types.keys()))
        raise ValueError(
            f"Input topic '{input_topic}' was not found in the bag. "
            f"Available topics: {available_topics}"
        )

    input_type = topic_types[input_topic]
    if input_type not in {OUTPUT_TYPE, LEGACY_INPUT_TYPE}:
        raise ValueError(
            f"Unsupported input topic type '{input_type}' for topic '{input_topic}'. "
            f"Supported types are '{OUTPUT_TYPE}' and '{LEGACY_INPUT_TYPE}'."
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag_path, storage_id=output_storage_id),
        converter_options,
    )

    topic_metadata = rosbag2_py.TopicMetadata(
        name=output_topic,
        type=OUTPUT_TYPE,
        serialization_format="cdr",
        offered_qos_profiles="",
    )
    writer.create_topic(topic_metadata)

    output_msg_cls = get_message(OUTPUT_TYPE)
    legacy_msg_cls = get_message(LEGACY_INPUT_TYPE)

    converted_count = 0
    scanned_count = 0
    legacy_fallback_count = 0
    failed_count = 0
    while reader.has_next():
        topic_name, serialized_message, timestamp = reader.read_next()
        scanned_count += 1
        if topic_name != input_topic:
            continue

        out_msg, used_fallback, failed = _deserialize_control_message(
            serialized_message, input_type, output_msg_cls, legacy_msg_cls
        )
        if failed:
            failed_count += 1
            continue
        if used_fallback:
            legacy_fallback_count += 1

        writer.write(output_topic, serialize_message(out_msg), timestamp)
        converted_count += 1

    print(f"Scanned messages: {scanned_count}")
    print(f"Converted messages from '{input_topic}': {converted_count}")
    print(f"Legacy fallback conversions: {legacy_fallback_count}")
    print(f"Failed message conversions: {failed_count}")
    print(f"Output bag written to: {output_bag_path}")

    return converted_count


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Offline converter for control commands in ROS 2 bags. "
            "Reads an input bag and writes a new bag without requiring playback."
        )
    )
    parser.add_argument(
        "input_bag",
        nargs="?",
        default=DEFAULT_INPUT_BAG,
        help=(
            "Path to the input bag (file or bag directory). "
            "Defaults to DEFAULT_INPUT_BAG at the top of this script."
        ),
    )
    parser.add_argument(
        "output_bag",
        nargs="?",
        default=DEFAULT_OUTPUT_BAG,
        help=(
            "Path for the output bag (must not exist). "
            "Defaults to DEFAULT_OUTPUT_BAG at the top of this script."
        ),
    )
    parser.add_argument(
        "--input-topic",
        default=DEFAULT_INPUT_TOPIC,
        help="Input topic to read from the input bag.",
    )
    parser.add_argument(
        "--output-topic",
        default=DEFAULT_OUTPUT_TOPIC,
        help="Output topic to write in the output bag.",
    )
    parser.add_argument(
        "--input-storage-id",
        default=DEFAULT_INPUT_STORAGE_ID,
        help="Input bag storage plugin id (for example: mcap, sqlite3).",
    )
    parser.add_argument(
        "--output-storage-id",
        default=DEFAULT_OUTPUT_STORAGE_ID,
        help="Output bag storage plugin id (for example: mcap, sqlite3).",
    )
    return parser.parse_args(args=args)


def main(args=None):
    parsed = parse_args(args)
    convert_bag(
        input_bag_path=parsed.input_bag,
        output_bag_path=parsed.output_bag,
        input_topic=parsed.input_topic,
        output_topic=parsed.output_topic,
        input_storage_id=parsed.input_storage_id,
        output_storage_id=parsed.output_storage_id,
    )


if __name__ == "__main__":
    main()
