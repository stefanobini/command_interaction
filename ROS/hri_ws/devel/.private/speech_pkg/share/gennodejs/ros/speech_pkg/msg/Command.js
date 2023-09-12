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

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.label = null;
      this.english = null;
      this.italian = null;
    }
    else {
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = 0;
      }
      if (initObj.hasOwnProperty('english')) {
        this.english = initObj.english
      }
      else {
        this.english = '';
      }
      if (initObj.hasOwnProperty('italian')) {
        this.italian = initObj.italian
      }
      else {
        this.italian = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [label]
    bufferOffset = _serializer.uint32(obj.label, buffer, bufferOffset);
    // Serialize message field [english]
    bufferOffset = _serializer.string(obj.english, buffer, bufferOffset);
    // Serialize message field [italian]
    bufferOffset = _serializer.string(obj.italian, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [label]
    data.label = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [english]
    data.english = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [italian]
    data.italian = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.english.length;
    length += object.italian.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'speech_pkg/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a48558522c26082d66040db118e6fd8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Identify the command detected by Speech/Command system, it contains the following information:
    
    # Numeric identifier associated with the command
    uint32 label
    # Textual description of the gesture, in English
    string english
    # Textual description of the gesture, in Italian
    string italian
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Command(null);
    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = 0
    }

    if (msg.english !== undefined) {
      resolved.english = msg.english;
    }
    else {
      resolved.english = ''
    }

    if (msg.italian !== undefined) {
      resolved.italian = msg.italian;
    }
    else {
      resolved.italian = ''
    }

    return resolved;
    }
};

module.exports = Command;
