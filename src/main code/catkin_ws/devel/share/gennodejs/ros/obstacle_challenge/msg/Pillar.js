// Auto-generated. Do not edit!

// (in-package obstacle_challenge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Pillar {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.centroid_x = null;
      this.centroid_y = null;
      this.distance = null;
      this.color = null;
    }
    else {
      if (initObj.hasOwnProperty('centroid_x')) {
        this.centroid_x = initObj.centroid_x
      }
      else {
        this.centroid_x = 0.0;
      }
      if (initObj.hasOwnProperty('centroid_y')) {
        this.centroid_y = initObj.centroid_y
      }
      else {
        this.centroid_y = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pillar
    // Serialize message field [centroid_x]
    bufferOffset = _serializer.float32(obj.centroid_x, buffer, bufferOffset);
    // Serialize message field [centroid_y]
    bufferOffset = _serializer.float32(obj.centroid_y, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [color]
    bufferOffset = _serializer.int8(obj.color, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pillar
    let len;
    let data = new Pillar(null);
    // Deserialize message field [centroid_x]
    data.centroid_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [centroid_y]
    data.centroid_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [color]
    data.color = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'obstacle_challenge/Pillar';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '577a76ead12b846a3fe1693414bc0bdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Pillar(null);
    if (msg.centroid_x !== undefined) {
      resolved.centroid_x = msg.centroid_x;
    }
    else {
      resolved.centroid_x = 0.0
    }

    if (msg.centroid_y !== undefined) {
      resolved.centroid_y = msg.centroid_y;
    }
    else {
      resolved.centroid_y = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.color !== undefined) {
      resolved.color = msg.color;
    }
    else {
      resolved.color = 0
    }

    return resolved;
    }
};

module.exports = Pillar;
