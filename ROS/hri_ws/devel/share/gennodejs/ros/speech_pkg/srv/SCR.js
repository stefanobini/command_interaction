// Auto-generated. Do not edit!

// (in-package speech_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SpeechData = require('../msg/SpeechData.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SCRRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new SpeechData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SCRRequest
    // Serialize message field [data]
    bufferOffset = SpeechData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SCRRequest
    let len;
    let data = new SCRRequest(null);
    // Deserialize message field [data]
    data.data = SpeechData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SpeechData.getMessageSize(object.data);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'speech_pkg/SCRRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21e1815396969a6f82c0ae7d856b83d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    speech_pkg/SpeechData data
    
    ================================================================================
    MSG: speech_pkg/SpeechData
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
    const resolved = new SCRRequest(null);
    if (msg.data !== undefined) {
      resolved.data = SpeechData.Resolve(msg.data)
    }
    else {
      resolved.data = new SpeechData()
    }

    return resolved;
    }
};

class SCRResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.probs = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('probs')) {
        this.probs = initObj.probs
      }
      else {
        this.probs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SCRResponse
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [probs]
    bufferOffset = _arraySerializer.float32(obj.probs, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SCRResponse
    let len;
    let data = new SCRResponse(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [probs]
    data.probs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.probs.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'speech_pkg/SCRResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b530570166a9025ca374b3a6285b3928';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    float32[] probs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SCRResponse(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.probs !== undefined) {
      resolved.probs = msg.probs;
    }
    else {
      resolved.probs = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SCRRequest,
  Response: SCRResponse,
  md5sum() { return '896849feb7849138871db12c46e37d25'; },
  datatype() { return 'speech_pkg/SCR'; }
};
