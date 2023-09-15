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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class narrowPathMarkersArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.marker_valid_array = null;
      this.marker_pose_array = null;
      this.marker_wrench_array = null;
      this.marker_goal_array = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('marker_valid_array')) {
        this.marker_valid_array = initObj.marker_valid_array
      }
      else {
        this.marker_valid_array = [];
      }
      if (initObj.hasOwnProperty('marker_pose_array')) {
        this.marker_pose_array = initObj.marker_pose_array
      }
      else {
        this.marker_pose_array = [];
      }
      if (initObj.hasOwnProperty('marker_wrench_array')) {
        this.marker_wrench_array = initObj.marker_wrench_array
      }
      else {
        this.marker_wrench_array = [];
      }
      if (initObj.hasOwnProperty('marker_goal_array')) {
        this.marker_goal_array = initObj.marker_goal_array
      }
      else {
        this.marker_goal_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type narrowPathMarkersArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [marker_valid_array]
    // Serialize the length for message field [marker_valid_array]
    bufferOffset = _serializer.uint32(obj.marker_valid_array.length, buffer, bufferOffset);
    obj.marker_valid_array.forEach((val) => {
      bufferOffset = std_msgs.msg.Bool.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [marker_pose_array]
    // Serialize the length for message field [marker_pose_array]
    bufferOffset = _serializer.uint32(obj.marker_pose_array.length, buffer, bufferOffset);
    obj.marker_pose_array.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [marker_wrench_array]
    // Serialize the length for message field [marker_wrench_array]
    bufferOffset = _serializer.uint32(obj.marker_wrench_array.length, buffer, bufferOffset);
    obj.marker_wrench_array.forEach((val) => {
      bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [marker_goal_array]
    // Serialize the length for message field [marker_goal_array]
    bufferOffset = _serializer.uint32(obj.marker_goal_array.length, buffer, bufferOffset);
    obj.marker_goal_array.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type narrowPathMarkersArray
    let len;
    let data = new narrowPathMarkersArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [marker_valid_array]
    // Deserialize array length for message field [marker_valid_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.marker_valid_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.marker_valid_array[i] = std_msgs.msg.Bool.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [marker_pose_array]
    // Deserialize array length for message field [marker_pose_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.marker_pose_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.marker_pose_array[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [marker_wrench_array]
    // Deserialize array length for message field [marker_wrench_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.marker_wrench_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.marker_wrench_array[i] = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [marker_goal_array]
    // Deserialize array length for message field [marker_goal_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.marker_goal_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.marker_goal_array[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.marker_valid_array.length;
    object.marker_pose_array.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.marker_wrench_array.forEach((val) => {
      length += geometry_msgs.msg.WrenchStamped.getMessageSize(val);
    });
    object.marker_goal_array.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iri_object_transportation_msgs/narrowPathMarkersArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e651f252e9d70e2375713d846ee9b4ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    std_msgs/Bool[] marker_valid_array
    geometry_msgs/PoseStamped[] marker_pose_array
    geometry_msgs/WrenchStamped[] marker_wrench_array
    geometry_msgs/PoseStamped[] marker_goal_array
    
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
    MSG: std_msgs/Bool
    bool data
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new narrowPathMarkersArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.marker_valid_array !== undefined) {
      resolved.marker_valid_array = new Array(msg.marker_valid_array.length);
      for (let i = 0; i < resolved.marker_valid_array.length; ++i) {
        resolved.marker_valid_array[i] = std_msgs.msg.Bool.Resolve(msg.marker_valid_array[i]);
      }
    }
    else {
      resolved.marker_valid_array = []
    }

    if (msg.marker_pose_array !== undefined) {
      resolved.marker_pose_array = new Array(msg.marker_pose_array.length);
      for (let i = 0; i < resolved.marker_pose_array.length; ++i) {
        resolved.marker_pose_array[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.marker_pose_array[i]);
      }
    }
    else {
      resolved.marker_pose_array = []
    }

    if (msg.marker_wrench_array !== undefined) {
      resolved.marker_wrench_array = new Array(msg.marker_wrench_array.length);
      for (let i = 0; i < resolved.marker_wrench_array.length; ++i) {
        resolved.marker_wrench_array[i] = geometry_msgs.msg.WrenchStamped.Resolve(msg.marker_wrench_array[i]);
      }
    }
    else {
      resolved.marker_wrench_array = []
    }

    if (msg.marker_goal_array !== undefined) {
      resolved.marker_goal_array = new Array(msg.marker_goal_array.length);
      for (let i = 0; i < resolved.marker_goal_array.length; ++i) {
        resolved.marker_goal_array[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.marker_goal_array[i]);
      }
    }
    else {
      resolved.marker_goal_array = []
    }

    return resolved;
    }
};

module.exports = narrowPathMarkersArray;
