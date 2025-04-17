export default {
    "name": "ROS IDL",
    "scopeName": "source.ros.idl",
    "fileTypes": ["msg", "srv", "action"],
    "patterns": [
      {
        "include": "#comments"
      },
      {
        "include": "#service_separator"
      },
      {
        "include": "#action_separator"
      },
      {
        "include": "#constant_declaration"
      },
      {
        "include": "#field_declaration"
      }
    ],
    "repository": {
      "comments": {
        "match": "#.*$",
        "name": "comment.line.number-sign.ros"
      },
      "service_separator": {
        "match": "^---$",
        "name": "keyword.operator.service.separator.ros"
      },
      "action_separator": {
        "match": "^---$",
        "name": "keyword.operator.action.separator.ros"
      },
      "constant_declaration": {
        "match": "^\\s*((string|bool|int8|uint8|int16|uint16|int32|uint32|int64|uint64|float32|float64|byte|char|time|duration))\\s+([A-Za-z][A-Za-z0-9_]*)\\s*=\\s*([^#\\s]+).*$",
        "captures": {
          "1": { "name": "storage.type.ros" },
          "3": { "name": "variable.other.constant.ros" },
          "4": { "name": "constant.other.ros" }
        }
      },
      "field_declaration": {
        "match": "^\\s*((string|bool|int8|uint8|int16|uint16|int32|uint32|int64|uint64|float32|float64|byte|char|time|duration)|([A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)?))\\s*(\\[\\s*([0-9]*)\\s*\\])?\\s+([A-Za-z][A-Za-z0-9_]*).*$",
        "captures": {
          "1": { "name": "storage.type.ros" },
          "2": { "name": "storage.type.primitive.ros" },
          "3": { "name": "storage.type.message.ros" },
          "5": { "name": "storage.modifier.array.ros" },
          "6": { "name": "constant.numeric.array.size.ros" },
          "7": { "name": "variable.other.member.ros" }
        }
      },
      "string_type": {
        "match": "\\b(string)\\b",
        "name": "storage.type.string.ros"
      },
      "numeric_types": {
        "match": "\\b(int8|uint8|int16|uint16|int32|uint32|int64|uint64|float32|float64|byte|char)\\b",
        "name": "storage.type.numeric.ros"
      },
      "time_types": {
        "match": "\\b(time|duration)\\b",
        "name": "storage.type.time.ros"
      },
      "bool_type": {
        "match": "\\b(bool)\\b",
        "name": "storage.type.boolean.ros"
      },
      "custom_type": {
        "match": "\\b([A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)?)\\b",
        "name": "storage.type.message.ros"
      },
      "array_declaration": {
        "match": "(\\[\\s*([0-9]*)\\s*\\])",
        "captures": {
          "1": { "name": "storage.modifier.array.ros" },
          "2": { "name": "constant.numeric.array.size.ros" }
        }
      }
    }
  };