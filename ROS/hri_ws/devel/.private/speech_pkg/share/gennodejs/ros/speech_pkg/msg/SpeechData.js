// Auto-generated. Do not edit!

// (in-package speech_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SpeechData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
      this.doa = null;
      this.start_time = null;
      this.end_time = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
      if (initObj.hasOwnProperty('doa')) {
        this.doa = initObj.doa
      }
      else {
        this.doa = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = 0.0;
      }
      if (initObj.hasOwnProperty('end_time')) {
        this.end_time = initObj.end_time
      }
      else {
        this.end_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpeechData
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int16(obj.data, buffer, bufferOffset, null);
    // Serialize message field [doa]
    bufferOffset = _serializer.int16(obj.doa, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.float64(obj.start_time, buffer, bufferOffset);
    // Serialize message field [end_time]
    bufferOffset = _serializer.float64(obj.end_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpeechData
    let len;
    let data = new SpeechData(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [doa]
    data.doa = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [end_time]
    data.end_time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.data.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'speech_pkg/SpeechData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2d258c719b9a4af008aff70e592c3a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16[] data
    int16 doa
    float64 start_time
    float64 end_time
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SpeechData(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    if (msg.doa !== undefined) {
      resolved.doa = msg.doa;
    }
    else {
      resolved.doa = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = 0.0
    }

    if (msg.end_time !== undefined) {
      resolved.end_time = msg.end_time;
    }
    else {
      resolved.end_time = 0.0
    }

    return resolved;
    }
};

module.exports = SpeechData;
