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

class ClassificationMSIRequest {
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
    // Serializes a message object of type ClassificationMSIRequest
    // Serialize message field [data]
    bufferOffset = SpeechData.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClassificationMSIRequest
    let len;
    let data = new ClassificationMSIRequest(null);
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
    return 'speech_pkg/ClassificationMSIRequest';
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
    const resolved = new ClassificationMSIRequest(null);
    if (msg.data !== undefined) {
      resolved.data = SpeechData.Resolve(msg.data)
    }
    else {
      resolved.data = new SpeechData()
    }

    return resolved;
    }
};

class ClassificationMSIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.intent = null;
      this.int_probs = null;
      this.explicit = null;
      this.exp_probs = null;
      this.implicit = null;
      this.imp_probs = null;
    }
    else {
      if (initObj.hasOwnProperty('intent')) {
        this.intent = initObj.intent
      }
      else {
        this.intent = 0;
      }
      if (initObj.hasOwnProperty('int_probs')) {
        this.int_probs = initObj.int_probs
      }
      else {
        this.int_probs = [];
      }
      if (initObj.hasOwnProperty('explicit')) {
        this.explicit = initObj.explicit
      }
      else {
        this.explicit = 0;
      }
      if (initObj.hasOwnProperty('exp_probs')) {
        this.exp_probs = initObj.exp_probs
      }
      else {
        this.exp_probs = [];
      }
      if (initObj.hasOwnProperty('implicit')) {
        this.implicit = initObj.implicit
      }
      else {
        this.implicit = 0;
      }
      if (initObj.hasOwnProperty('imp_probs')) {
        this.imp_probs = initObj.imp_probs
      }
      else {
        this.imp_probs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClassificationMSIResponse
    // Serialize message field [intent]
    bufferOffset = _serializer.int8(obj.intent, buffer, bufferOffset);
    // Serialize message field [int_probs]
    bufferOffset = _arraySerializer.float32(obj.int_probs, buffer, bufferOffset, null);
    // Serialize message field [explicit]
    bufferOffset = _serializer.int8(obj.explicit, buffer, bufferOffset);
    // Serialize message field [exp_probs]
    bufferOffset = _arraySerializer.float32(obj.exp_probs, buffer, bufferOffset, null);
    // Serialize message field [implicit]
    bufferOffset = _serializer.int8(obj.implicit, buffer, bufferOffset);
    // Serialize message field [imp_probs]
    bufferOffset = _arraySerializer.float32(obj.imp_probs, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClassificationMSIResponse
    let len;
    let data = new ClassificationMSIResponse(null);
    // Deserialize message field [intent]
    data.intent = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [int_probs]
    data.int_probs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [explicit]
    data.explicit = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [exp_probs]
    data.exp_probs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [implicit]
    data.implicit = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [imp_probs]
    data.imp_probs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.int_probs.length;
    length += 4 * object.exp_probs.length;
    length += 4 * object.imp_probs.length;
    return length + 15;
  }

  static datatype() {
    // Returns string type for a service object
    return 'speech_pkg/ClassificationMSIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93f46f730242eecddbf7b7d028f5e49b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 intent
    float32[] int_probs
    int8 explicit
    float32[] exp_probs
    int8 implicit
    float32[] imp_probs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ClassificationMSIResponse(null);
    if (msg.intent !== undefined) {
      resolved.intent = msg.intent;
    }
    else {
      resolved.intent = 0
    }

    if (msg.int_probs !== undefined) {
      resolved.int_probs = msg.int_probs;
    }
    else {
      resolved.int_probs = []
    }

    if (msg.explicit !== undefined) {
      resolved.explicit = msg.explicit;
    }
    else {
      resolved.explicit = 0
    }

    if (msg.exp_probs !== undefined) {
      resolved.exp_probs = msg.exp_probs;
    }
    else {
      resolved.exp_probs = []
    }

    if (msg.implicit !== undefined) {
      resolved.implicit = msg.implicit;
    }
    else {
      resolved.implicit = 0
    }

    if (msg.imp_probs !== undefined) {
      resolved.imp_probs = msg.imp_probs;
    }
    else {
      resolved.imp_probs = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ClassificationMSIRequest,
  Response: ClassificationMSIResponse,
  md5sum() { return '86bdd504432c8f4cd29e4aa496bd70a0'; },
  datatype() { return 'speech_pkg/ClassificationMSI'; }
};
