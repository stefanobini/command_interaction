// Auto-generated. Do not edit!

// (in-package speech_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class IntentIRI {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fsr_values = null;
      this.sw_values = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fsr_values')) {
        this.fsr_values = initObj.fsr_values
      }
      else {
        this.fsr_values = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('sw_values')) {
        this.sw_values = initObj.sw_values
      }
      else {
        this.sw_values = new Array(5).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IntentIRI
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [fsr_values] has the right length
    if (obj.fsr_values.length !== 5) {
      throw new Error('Unable to serialize array field fsr_values - length must be 5')
    }
    // Serialize message field [fsr_values]
    bufferOffset = _arraySerializer.float64(obj.fsr_values, buffer, bufferOffset, 5);
    // Check that the constant length array field [sw_values] has the right length
    if (obj.sw_values.length !== 5) {
      throw new Error('Unable to serialize array field sw_values - length must be 5')
    }
    // Serialize message field [sw_values]
    bufferOffset = _arraySerializer.bool(obj.sw_values, buffer, bufferOffset, 5);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IntentIRI
    let len;
    let data = new IntentIRI(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fsr_values]
    data.fsr_values = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    // Deserialize message field [sw_values]
    data.sw_values = _arrayDeserializer.bool(buffer, bufferOffset, 5)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 45;
  }

  static datatype() {
    // Returns string type for a message object
    return 'speech_pkg/IntentIRI';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62dc5f03f1a505652316491d926de860';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[5] fsr_values
    bool[5] sw_values
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
    const resolved = new IntentIRI(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fsr_values !== undefined) {
      resolved.fsr_values = msg.fsr_values;
    }
    else {
      resolved.fsr_values = new Array(5).fill(0)
    }

    if (msg.sw_values !== undefined) {
      resolved.sw_values = msg.sw_values;
    }
    else {
      resolved.sw_values = new Array(5).fill(0)
    }

    return resolved;
    }
};

module.exports = IntentIRI;
