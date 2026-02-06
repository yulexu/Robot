// Auto-generated. Do not edit!

// (in-package feetech_controls.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class readjointvaluesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.idstart = null;
      this.idend = null;
    }
    else {
      if (initObj.hasOwnProperty('idstart')) {
        this.idstart = initObj.idstart
      }
      else {
        this.idstart = 0;
      }
      if (initObj.hasOwnProperty('idend')) {
        this.idend = initObj.idend
      }
      else {
        this.idend = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type readjointvaluesRequest
    // Serialize message field [idstart]
    bufferOffset = _serializer.int64(obj.idstart, buffer, bufferOffset);
    // Serialize message field [idend]
    bufferOffset = _serializer.int64(obj.idend, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type readjointvaluesRequest
    let len;
    let data = new readjointvaluesRequest(null);
    // Deserialize message field [idstart]
    data.idstart = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [idend]
    data.idend = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'feetech_controls/readjointvaluesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3686e6221b7224d0bb35c49c32d8e79d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 idstart
    int64 idend
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new readjointvaluesRequest(null);
    if (msg.idstart !== undefined) {
      resolved.idstart = msg.idstart;
    }
    else {
      resolved.idstart = 0
    }

    if (msg.idend !== undefined) {
      resolved.idend = msg.idend;
    }
    else {
      resolved.idend = 0
    }

    return resolved;
    }
};

class readjointvaluesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jointreadvalues = null;
    }
    else {
      if (initObj.hasOwnProperty('jointreadvalues')) {
        this.jointreadvalues = initObj.jointreadvalues
      }
      else {
        this.jointreadvalues = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type readjointvaluesResponse
    // Serialize message field [jointreadvalues]
    bufferOffset = _arraySerializer.float64(obj.jointreadvalues, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type readjointvaluesResponse
    let len;
    let data = new readjointvaluesResponse(null);
    // Deserialize message field [jointreadvalues]
    data.jointreadvalues = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.jointreadvalues.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'feetech_controls/readjointvaluesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7e1ffb0293b88553551dedfd053500a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] jointreadvalues
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new readjointvaluesResponse(null);
    if (msg.jointreadvalues !== undefined) {
      resolved.jointreadvalues = msg.jointreadvalues;
    }
    else {
      resolved.jointreadvalues = []
    }

    return resolved;
    }
};

module.exports = {
  Request: readjointvaluesRequest,
  Response: readjointvaluesResponse,
  md5sum() { return '21a5fa9eeca5ec0f0f497e63ffbb5aa1'; },
  datatype() { return 'feetech_controls/readjointvalues'; }
};
