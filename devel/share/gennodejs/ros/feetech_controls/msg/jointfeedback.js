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

class jointfeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jointvalues = null;
    }
    else {
      if (initObj.hasOwnProperty('jointvalues')) {
        this.jointvalues = initObj.jointvalues
      }
      else {
        this.jointvalues = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type jointfeedback
    // Serialize message field [jointvalues]
    bufferOffset = _arraySerializer.float64(obj.jointvalues, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type jointfeedback
    let len;
    let data = new jointfeedback(null);
    // Deserialize message field [jointvalues]
    data.jointvalues = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.jointvalues.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'feetech_controls/jointfeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9d4993eb244bba482f1be1b903382b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] jointvalues
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new jointfeedback(null);
    if (msg.jointvalues !== undefined) {
      resolved.jointvalues = msg.jointvalues;
    }
    else {
      resolved.jointvalues = []
    }

    return resolved;
    }
};

module.exports = jointfeedback;
