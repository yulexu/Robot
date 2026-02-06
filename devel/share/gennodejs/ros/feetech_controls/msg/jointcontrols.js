// Auto-generated. Do not edit!

// (in-package feetech_controls.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class jointcontrols {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.jointvalues = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('jointvalues')) {
        this.jointvalues = initObj.jointvalues
      }
      else {
        this.jointvalues = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type jointcontrols
    // Serialize message field [id]
    bufferOffset = _serializer.int16(obj.id, buffer, bufferOffset);
    // Serialize message field [jointvalues]
    bufferOffset = _serializer.float64(obj.jointvalues, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type jointcontrols
    let len;
    let data = new jointcontrols(null);
    // Deserialize message field [id]
    data.id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [jointvalues]
    data.jointvalues = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'feetech_controls/jointcontrols';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7988f738ee37fb27a094c2d2252a41df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 id
    float64 jointvalues
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new jointcontrols(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.jointvalues !== undefined) {
      resolved.jointvalues = msg.jointvalues;
    }
    else {
      resolved.jointvalues = 0.0
    }

    return resolved;
    }
};

module.exports = jointcontrols;
