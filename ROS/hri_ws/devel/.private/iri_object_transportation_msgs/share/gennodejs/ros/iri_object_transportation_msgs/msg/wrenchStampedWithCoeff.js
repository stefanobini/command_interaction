// Auto-generated. Do not edit!

// (in-package iri_object_transportation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class wrenchStampedWithCoeff {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.external_force = null;
      this.coefficient = null;
    }
    else {
      if (initObj.hasOwnProperty('external_force')) {
        this.external_force = initObj.external_force
      }
      else {
        this.external_force = new geometry_msgs.msg.WrenchStamped();
      }
      if (initObj.hasOwnProperty('coefficient')) {
        this.coefficient = initObj.coefficient
      }
      else {
        this.coefficient = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wrenchStampedWithCoeff
    // Serialize message field [external_force]
    bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(obj.external_force, buffer, bufferOffset);
    // Serialize message field [coefficient]
    bufferOffset = _serializer.float32(obj.coefficient, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wrenchStampedWithCoeff
    let len;
    let data = new wrenchStampedWithCoeff(null);
    // Deserialize message field [external_force]
    data.external_force = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [coefficient]
    data.coefficient = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.WrenchStamped.getMessageSize(object.external_force);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iri_object_transportation_msgs/wrenchStampedWithCoeff';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f94a203c7491df5185f74fa3237bfc7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/WrenchStamped external_force
    float32 coefficient
    
    ================================================================================
    MSG: geometry_msgs/WrenchStamped
    # A wrench with reference coordinate frame and timestamp
    Header header
    Wrench wrench
    
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
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
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
    const resolved = new wrenchStampedWithCoeff(null);
    if (msg.external_force !== undefined) {
      resolved.external_force = geometry_msgs.msg.WrenchStamped.Resolve(msg.external_force)
    }
    else {
      resolved.external_force = new geometry_msgs.msg.WrenchStamped()
    }

    if (msg.coefficient !== undefined) {
      resolved.coefficient = msg.coefficient;
    }
    else {
      resolved.coefficient = 0.0
    }

    return resolved;
    }
};

module.exports = wrenchStampedWithCoeff;
