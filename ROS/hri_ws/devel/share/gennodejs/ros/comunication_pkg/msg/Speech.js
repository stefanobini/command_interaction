// Auto-generated. Do not edit!

// (in-package comunication_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Command = require('./Command.js');

//-----------------------------------------------------------

class Speech {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.type = null;
      this.timestamp = null;
      this.command = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = '';
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = new Command();
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Speech
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.string(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = Command.serialize(obj.command, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Speech
    let len;
    let data = new Speech(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = Command.deserialize(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.type.length;
    length += object.timestamp.length;
    length += Command.getMessageSize(object.command);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'comunication_pkg/Speech';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c31bef49fddac6ccf19a75e92a735ec5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 
    
    # Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)
    string id
    # Identify the Entity/topic type (Speech)
    string type
    #Two-integer timestamp that is expressed as:
    # * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    string timestamp
    # Identify the command detected by Speech/Command system, it contains the following information:
    # * Numeric identifier associated with the command
    # * Textual description of the gesture, in English
    # * Textual description of the gesture, in Italian
    Command command
    # Value between 0 and 1 which indicates the reliability in the command identification and classification
    float32 confidence
    ================================================================================
    MSG: comunication_pkg/Command
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
    const resolved = new Speech(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = ''
    }

    if (msg.command !== undefined) {
      resolved.command = Command.Resolve(msg.command)
    }
    else {
      resolved.command = new Command()
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    return resolved;
    }
};

module.exports = Speech;
