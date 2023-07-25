// Auto-generated. Do not edit!

// (in-package iri_object_transportation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class localForcesCoefficients {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal_coefficient = null;
      this.attractor_coefficient = null;
      this.obstacles_coefficients = null;
      this.total_coefficient = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal_coefficient')) {
        this.goal_coefficient = initObj.goal_coefficient
      }
      else {
        this.goal_coefficient = 0.0;
      }
      if (initObj.hasOwnProperty('attractor_coefficient')) {
        this.attractor_coefficient = initObj.attractor_coefficient
      }
      else {
        this.attractor_coefficient = 0.0;
      }
      if (initObj.hasOwnProperty('obstacles_coefficients')) {
        this.obstacles_coefficients = initObj.obstacles_coefficients
      }
      else {
        this.obstacles_coefficients = [];
      }
      if (initObj.hasOwnProperty('total_coefficient')) {
        this.total_coefficient = initObj.total_coefficient
      }
      else {
        this.total_coefficient = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type localForcesCoefficients
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal_coefficient]
    bufferOffset = _serializer.float32(obj.goal_coefficient, buffer, bufferOffset);
    // Serialize message field [attractor_coefficient]
    bufferOffset = _serializer.float32(obj.attractor_coefficient, buffer, bufferOffset);
    // Serialize message field [obstacles_coefficients]
    bufferOffset = _arraySerializer.float32(obj.obstacles_coefficients, buffer, bufferOffset, null);
    // Serialize message field [total_coefficient]
    bufferOffset = _serializer.float32(obj.total_coefficient, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type localForcesCoefficients
    let len;
    let data = new localForcesCoefficients(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_coefficient]
    data.goal_coefficient = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [attractor_coefficient]
    data.attractor_coefficient = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obstacles_coefficients]
    data.obstacles_coefficients = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [total_coefficient]
    data.total_coefficient = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.obstacles_coefficients.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iri_object_transportation_msgs/localForcesCoefficients';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9b5321a9227a9197e9c265bb9376d55';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 goal_coefficient
    float32 attractor_coefficient
    float32[] obstacles_coefficients
    float32 total_coefficient
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new localForcesCoefficients(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal_coefficient !== undefined) {
      resolved.goal_coefficient = msg.goal_coefficient;
    }
    else {
      resolved.goal_coefficient = 0.0
    }

    if (msg.attractor_coefficient !== undefined) {
      resolved.attractor_coefficient = msg.attractor_coefficient;
    }
    else {
      resolved.attractor_coefficient = 0.0
    }

    if (msg.obstacles_coefficients !== undefined) {
      resolved.obstacles_coefficients = msg.obstacles_coefficients;
    }
    else {
      resolved.obstacles_coefficients = []
    }

    if (msg.total_coefficient !== undefined) {
      resolved.total_coefficient = msg.total_coefficient;
    }
    else {
      resolved.total_coefficient = 0.0
    }

    return resolved;
    }
};

module.exports = localForcesCoefficients;
