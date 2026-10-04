#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

namespace invictasim::config {

/**
 * @brief Read an optional scalar from a YAML node, keeping the current value when it is absent.
 *
 * Lets every adapter config file stay partial: only the keys a user wants to override need to be
 * present, everything else keeps the default baked into the adapter.
 *
 * @param node Parent node, may be invalid.
 * @param key Key to read.
 * @param value Value to overwrite when the key exists.
 */
template <typename T>
void load_optional(const YAML::Node& node, const std::string& key, T& value) {
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}

}  // namespace invictasim::config
