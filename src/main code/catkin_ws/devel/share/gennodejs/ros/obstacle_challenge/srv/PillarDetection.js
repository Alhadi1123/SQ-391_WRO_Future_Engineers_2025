// Auto-generated. Do not edit!

// (in-package obstacle_challenge.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let Pillar = require('../msg/Pillar.js');

//-----------------------------------------------------------

class PillarDetectionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PillarDetectionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PillarDetectionRequest
    let len;
    let data = new PillarDetectionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'obstacle_challenge/PillarDetectionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request (empty since client just requests data)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PillarDetectionRequest(null);
    return resolved;
    }
};

class PillarDetectionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pillars = null;
    }
    else {
      if (initObj.hasOwnProperty('pillars')) {
        this.pillars = initObj.pillars
      }
      else {
        this.pillars = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PillarDetectionResponse
    // Serialize message field [pillars]
    // Serialize the length for message field [pillars]
    bufferOffset = _serializer.uint32(obj.pillars.length, buffer, bufferOffset);
    obj.pillars.forEach((val) => {
      bufferOffset = Pillar.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PillarDetectionResponse
    let len;
    let data = new PillarDetectionResponse(null);
    // Deserialize message field [pillars]
    // Deserialize array length for message field [pillars]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pillars = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pillars[i] = Pillar.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 13 * object.pillars.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'obstacle_challenge/PillarDetectionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05c01fe013cbbc393a81b8e5fc3bc0a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    Pillar[] pillars
    
    
    
    
    ================================================================================
    MSG: obstacle_challenge/Pillar
    float32 centroid_x
    float32 centroid_y
    float32 distance
    int8 color
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PillarDetectionResponse(null);
    if (msg.pillars !== undefined) {
      resolved.pillars = new Array(msg.pillars.length);
      for (let i = 0; i < resolved.pillars.length; ++i) {
        resolved.pillars[i] = Pillar.Resolve(msg.pillars[i]);
      }
    }
    else {
      resolved.pillars = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PillarDetectionRequest,
  Response: PillarDetectionResponse,
  md5sum() { return '05c01fe013cbbc393a81b8e5fc3bc0a8'; },
  datatype() { return 'obstacle_challenge/PillarDetection'; }
};
