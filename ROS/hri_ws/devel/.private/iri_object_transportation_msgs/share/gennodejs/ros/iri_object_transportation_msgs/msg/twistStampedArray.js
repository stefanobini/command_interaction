// Auto-generated. Do not edit!

// (in-package iri_object_transportation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let twistStamped = require('./twistStamped.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class twistStampedArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.twist_array = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('twist_array')) {
        this.twist_array = initObj.twist_array
      }
      else {
        this.twist_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type twistStampedArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [twist_array]
    // Serialize the length for message field [twist_array]
    bufferOffset = _serializer.uint32(obj.twist_array.length, buffer, bufferOffset);
    obj.twist_array.forEach((val) => {
      bufferOffset = twistStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type twistStampedArray
    let len;
    let data = new twistStampedArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist_array]
    // Deserialize array length for message field [twist_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.twist_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.twist_array[i] = twistStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.twist_array.forEach((val) => {
      length += twistStamped.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iri_object_transportation_msgs/twistStampedArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03e348c69a5cfa2196a94fd09cb86416';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    iri_object_transportation_msgs/twistStamped[] twist_array
    
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
    
    ================================================================================
    MSG: iri_object_transportation_msgs/twistStamped
    std_msgs/Header header
    geometry_msgs/Twist twist
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new twistStampedArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.twist_array !== undefined) {
      resolved.twist_array = new Array(msg.twist_array.length);
      for (let i = 0; i < resolved.twist_array.length; ++i) {
        resolved.twist_array[i] = twistStamped.Resolve(msg.twist_array[i]);
      }
    }
    else {
      resolved.twist_array = []
    }

    return resolved;
    }
};

module.exports = twistStampedArray;
