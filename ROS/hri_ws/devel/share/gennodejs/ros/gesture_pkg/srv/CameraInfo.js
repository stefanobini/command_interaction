// Auto-generated. Do not edit!

// (in-package gesture_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CameraInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraInfoRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraInfoRequest
    let len;
    let data = new CameraInfoRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gesture_pkg/CameraInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraInfoRequest(null);
    return resolved;
    }
};

class CameraInfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.width = null;
      this.height = null;
      this.hfov = null;
      this.vfov = null;
    }
    else {
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('hfov')) {
        this.hfov = initObj.hfov
      }
      else {
        this.hfov = 0.0;
      }
      if (initObj.hasOwnProperty('vfov')) {
        this.vfov = initObj.vfov
      }
      else {
        this.vfov = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraInfoResponse
    // Serialize message field [width]
    bufferOffset = _serializer.int64(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.int64(obj.height, buffer, bufferOffset);
    // Serialize message field [hfov]
    bufferOffset = _serializer.float32(obj.hfov, buffer, bufferOffset);
    // Serialize message field [vfov]
    bufferOffset = _serializer.float32(obj.vfov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraInfoResponse
    let len;
    let data = new CameraInfoResponse(null);
    // Deserialize message field [width]
    data.width = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [hfov]
    data.hfov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vfov]
    data.vfov = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gesture_pkg/CameraInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e0f6cc9c6dfad0bee4253b15158a925';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 width
    int64 height
    float32 hfov
    float32 vfov
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraInfoResponse(null);
    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.hfov !== undefined) {
      resolved.hfov = msg.hfov;
    }
    else {
      resolved.hfov = 0.0
    }

    if (msg.vfov !== undefined) {
      resolved.vfov = msg.vfov;
    }
    else {
      resolved.vfov = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: CameraInfoRequest,
  Response: CameraInfoResponse,
  md5sum() { return '1e0f6cc9c6dfad0bee4253b15158a925'; },
  datatype() { return 'gesture_pkg/CameraInfo'; }
};
