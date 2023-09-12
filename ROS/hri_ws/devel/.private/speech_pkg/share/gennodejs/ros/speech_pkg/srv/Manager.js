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

class ManagerRequest {
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
    // Serializes a message object of type ManagerRequest
    // Serialize message field [data]
    bufferOffset = SpeechData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ManagerRequest
    let len;
    let data = new ManagerRequest(null);
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
    return 'speech_pkg/ManagerRequest';
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
    const resolved = new ManagerRequest(null);
    if (msg.data !== undefined) {
      resolved.data = SpeechData.Resolve(msg.data)
    }
    else {
      resolved.data = new SpeechData()
    }

    return resolved;
    }
};

class ManagerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag = null;
    }
    else {
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ManagerResponse
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ManagerResponse
    let len;
    let data = new ManagerResponse(null);
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'speech_pkg/ManagerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24842bc754e0f5cc982338eca1269251';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool flag
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ManagerResponse(null);
    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ManagerRequest,
  Response: ManagerResponse,
  md5sum() { return '426efa73e6c8d460d7543a8bf442644f'; },
  datatype() { return 'speech_pkg/Manager'; }
};
