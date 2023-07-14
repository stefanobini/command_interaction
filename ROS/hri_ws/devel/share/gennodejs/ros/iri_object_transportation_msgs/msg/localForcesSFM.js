// Auto-generated. Do not edit!

// (in-package iri_object_transportation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let wrenchStampedArray = require('./wrenchStampedArray.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class localForcesSFM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal_force = null;
      this.attractor_force = null;
      this.obstacles_forces = null;
      this.total_force = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal_force')) {
        this.goal_force = initObj.goal_force
      }
      else {
        this.goal_force = new geometry_msgs.msg.WrenchStamped();
      }
      if (initObj.hasOwnProperty('attractor_force')) {
        this.attractor_force = initObj.attractor_force
      }
      else {
        this.attractor_force = new geometry_msgs.msg.WrenchStamped();
      }
      if (initObj.hasOwnProperty('obstacles_forces')) {
        this.obstacles_forces = initObj.obstacles_forces
      }
      else {
        this.obstacles_forces = new wrenchStampedArray();
      }
      if (initObj.hasOwnProperty('total_force')) {
        this.total_force = initObj.total_force
      }
      else {
        this.total_force = new geometry_msgs.msg.WrenchStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type localForcesSFM
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal_force]
    bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(obj.goal_force, buffer, bufferOffset);
    // Serialize message field [attractor_force]
    bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(obj.attractor_force, buffer, bufferOffset);
    // Serialize message field [obstacles_forces]
    bufferOffset = wrenchStampedArray.serialize(obj.obstacles_forces, buffer, bufferOffset);
    // Serialize message field [total_force]
    bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(obj.total_force, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type localForcesSFM
    let len;
    let data = new localForcesSFM(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_force]
    data.goal_force = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [attractor_force]
    data.attractor_force = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacles_forces]
    data.obstacles_forces = wrenchStampedArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [total_force]
    data.total_force = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += geometry_msgs.msg.WrenchStamped.getMessageSize(object.goal_force);
    length += geometry_msgs.msg.WrenchStamped.getMessageSize(object.attractor_force);
    length += wrenchStampedArray.getMessageSize(object.obstacles_forces);
    length += geometry_msgs.msg.WrenchStamped.getMessageSize(object.total_force);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iri_object_transportation_msgs/localForcesSFM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1cd253465458d5a2c178d26ef88bcf8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/WrenchStamped goal_force
    geometry_msgs/WrenchStamped attractor_force
    iri_object_transportation_msgs/wrenchStampedArray obstacles_forces
    geometry_msgs/WrenchStamped total_force
    
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
    MSG: geometry_msgs/WrenchStamped
    # A wrench with reference coordinate frame and timestamp
    Header header
    Wrench wrench
    
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
    ================================================================================
    MSG: iri_object_transportation_msgs/wrenchStampedArray
    Header header
    geometry_msgs/WrenchStamped[] wrench_array
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new localForcesSFM(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal_force !== undefined) {
      resolved.goal_force = geometry_msgs.msg.WrenchStamped.Resolve(msg.goal_force)
    }
    else {
      resolved.goal_force = new geometry_msgs.msg.WrenchStamped()
    }

    if (msg.attractor_force !== undefined) {
      resolved.attractor_force = geometry_msgs.msg.WrenchStamped.Resolve(msg.attractor_force)
    }
    else {
      resolved.attractor_force = new geometry_msgs.msg.WrenchStamped()
    }

    if (msg.obstacles_forces !== undefined) {
      resolved.obstacles_forces = wrenchStampedArray.Resolve(msg.obstacles_forces)
    }
    else {
      resolved.obstacles_forces = new wrenchStampedArray()
    }

    if (msg.total_force !== undefined) {
      resolved.total_force = geometry_msgs.msg.WrenchStamped.Resolve(msg.total_force)
    }
    else {
      resolved.total_force = new geometry_msgs.msg.WrenchStamped()
    }

    return resolved;
    }
};

module.exports = localForcesSFM;
