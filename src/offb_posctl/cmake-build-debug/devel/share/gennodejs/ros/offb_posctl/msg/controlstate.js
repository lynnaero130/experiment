// Auto-generated. Do not edit!

// (in-package offb_posctl.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class controlstate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.discrepointpersecond = null;
      this.inicounter = null;
      this.arraylength = null;
      this.wall_z = null;
      this.wall_y = null;
      this.parabolictime = null;
      this.thrustarray = null;
      this.tauarray = null;
      this.phiarray = null;
      this.thetaarray = null;
      this.stateXarray = null;
      this.stateYarray = null;
      this.stateZarray = null;
      this.stateVXarray = null;
      this.stateVYarray = null;
      this.stateVZarray = null;
      this.stateAXarray = null;
      this.stateAYarray = null;
      this.stateAZarray = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('discrepointpersecond')) {
        this.discrepointpersecond = initObj.discrepointpersecond
      }
      else {
        this.discrepointpersecond = 0;
      }
      if (initObj.hasOwnProperty('inicounter')) {
        this.inicounter = initObj.inicounter
      }
      else {
        this.inicounter = 0;
      }
      if (initObj.hasOwnProperty('arraylength')) {
        this.arraylength = initObj.arraylength
      }
      else {
        this.arraylength = 0;
      }
      if (initObj.hasOwnProperty('wall_z')) {
        this.wall_z = initObj.wall_z
      }
      else {
        this.wall_z = 0.0;
      }
      if (initObj.hasOwnProperty('wall_y')) {
        this.wall_y = initObj.wall_y
      }
      else {
        this.wall_y = 0.0;
      }
      if (initObj.hasOwnProperty('parabolictime')) {
        this.parabolictime = initObj.parabolictime
      }
      else {
        this.parabolictime = 0.0;
      }
      if (initObj.hasOwnProperty('thrustarray')) {
        this.thrustarray = initObj.thrustarray
      }
      else {
        this.thrustarray = [];
      }
      if (initObj.hasOwnProperty('tauarray')) {
        this.tauarray = initObj.tauarray
      }
      else {
        this.tauarray = [];
      }
      if (initObj.hasOwnProperty('phiarray')) {
        this.phiarray = initObj.phiarray
      }
      else {
        this.phiarray = [];
      }
      if (initObj.hasOwnProperty('thetaarray')) {
        this.thetaarray = initObj.thetaarray
      }
      else {
        this.thetaarray = [];
      }
      if (initObj.hasOwnProperty('stateXarray')) {
        this.stateXarray = initObj.stateXarray
      }
      else {
        this.stateXarray = [];
      }
      if (initObj.hasOwnProperty('stateYarray')) {
        this.stateYarray = initObj.stateYarray
      }
      else {
        this.stateYarray = [];
      }
      if (initObj.hasOwnProperty('stateZarray')) {
        this.stateZarray = initObj.stateZarray
      }
      else {
        this.stateZarray = [];
      }
      if (initObj.hasOwnProperty('stateVXarray')) {
        this.stateVXarray = initObj.stateVXarray
      }
      else {
        this.stateVXarray = [];
      }
      if (initObj.hasOwnProperty('stateVYarray')) {
        this.stateVYarray = initObj.stateVYarray
      }
      else {
        this.stateVYarray = [];
      }
      if (initObj.hasOwnProperty('stateVZarray')) {
        this.stateVZarray = initObj.stateVZarray
      }
      else {
        this.stateVZarray = [];
      }
      if (initObj.hasOwnProperty('stateAXarray')) {
        this.stateAXarray = initObj.stateAXarray
      }
      else {
        this.stateAXarray = [];
      }
      if (initObj.hasOwnProperty('stateAYarray')) {
        this.stateAYarray = initObj.stateAYarray
      }
      else {
        this.stateAYarray = [];
      }
      if (initObj.hasOwnProperty('stateAZarray')) {
        this.stateAZarray = initObj.stateAZarray
      }
      else {
        this.stateAZarray = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type controlstate
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [discrepointpersecond]
    bufferOffset = _serializer.int16(obj.discrepointpersecond, buffer, bufferOffset);
    // Serialize message field [inicounter]
    bufferOffset = _serializer.int16(obj.inicounter, buffer, bufferOffset);
    // Serialize message field [arraylength]
    bufferOffset = _serializer.int16(obj.arraylength, buffer, bufferOffset);
    // Serialize message field [wall_z]
    bufferOffset = _serializer.float32(obj.wall_z, buffer, bufferOffset);
    // Serialize message field [wall_y]
    bufferOffset = _serializer.float32(obj.wall_y, buffer, bufferOffset);
    // Serialize message field [parabolictime]
    bufferOffset = _serializer.float32(obj.parabolictime, buffer, bufferOffset);
    // Serialize message field [thrustarray]
    bufferOffset = _arraySerializer.float32(obj.thrustarray, buffer, bufferOffset, null);
    // Serialize message field [tauarray]
    bufferOffset = _arraySerializer.float32(obj.tauarray, buffer, bufferOffset, null);
    // Serialize message field [phiarray]
    bufferOffset = _arraySerializer.float32(obj.phiarray, buffer, bufferOffset, null);
    // Serialize message field [thetaarray]
    bufferOffset = _arraySerializer.float32(obj.thetaarray, buffer, bufferOffset, null);
    // Serialize message field [stateXarray]
    bufferOffset = _arraySerializer.float32(obj.stateXarray, buffer, bufferOffset, null);
    // Serialize message field [stateYarray]
    bufferOffset = _arraySerializer.float32(obj.stateYarray, buffer, bufferOffset, null);
    // Serialize message field [stateZarray]
    bufferOffset = _arraySerializer.float32(obj.stateZarray, buffer, bufferOffset, null);
    // Serialize message field [stateVXarray]
    bufferOffset = _arraySerializer.float32(obj.stateVXarray, buffer, bufferOffset, null);
    // Serialize message field [stateVYarray]
    bufferOffset = _arraySerializer.float32(obj.stateVYarray, buffer, bufferOffset, null);
    // Serialize message field [stateVZarray]
    bufferOffset = _arraySerializer.float32(obj.stateVZarray, buffer, bufferOffset, null);
    // Serialize message field [stateAXarray]
    bufferOffset = _arraySerializer.float32(obj.stateAXarray, buffer, bufferOffset, null);
    // Serialize message field [stateAYarray]
    bufferOffset = _arraySerializer.float32(obj.stateAYarray, buffer, bufferOffset, null);
    // Serialize message field [stateAZarray]
    bufferOffset = _arraySerializer.float32(obj.stateAZarray, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controlstate
    let len;
    let data = new controlstate(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [discrepointpersecond]
    data.discrepointpersecond = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [inicounter]
    data.inicounter = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [arraylength]
    data.arraylength = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [wall_z]
    data.wall_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wall_y]
    data.wall_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [parabolictime]
    data.parabolictime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thrustarray]
    data.thrustarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [tauarray]
    data.tauarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [phiarray]
    data.phiarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [thetaarray]
    data.thetaarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateXarray]
    data.stateXarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateYarray]
    data.stateYarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateZarray]
    data.stateZarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateVXarray]
    data.stateVXarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateVYarray]
    data.stateVYarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateVZarray]
    data.stateVZarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateAXarray]
    data.stateAXarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateAYarray]
    data.stateAYarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateAZarray]
    data.stateAZarray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.thrustarray.length;
    length += 4 * object.tauarray.length;
    length += 4 * object.phiarray.length;
    length += 4 * object.thetaarray.length;
    length += 4 * object.stateXarray.length;
    length += 4 * object.stateYarray.length;
    length += 4 * object.stateZarray.length;
    length += 4 * object.stateVXarray.length;
    length += 4 * object.stateVYarray.length;
    length += 4 * object.stateVZarray.length;
    length += 4 * object.stateAXarray.length;
    length += 4 * object.stateAYarray.length;
    length += 4 * object.stateAZarray.length;
    return length + 70;
  }

  static datatype() {
    // Returns string type for a message object
    return 'offb_posctl/controlstate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa885756731fe76aac6c7868ca11c7f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int16 discrepointpersecond
    int16 inicounter
    int16 arraylength
    float32 wall_z
    float32 wall_y
    float32 parabolictime
    float32[] thrustarray
    float32[] tauarray
    float32[] phiarray
    float32[] thetaarray
    float32[] stateXarray
    float32[] stateYarray
    float32[] stateZarray
    float32[] stateVXarray
    float32[] stateVYarray
    float32[] stateVZarray
    float32[] stateAXarray
    float32[] stateAYarray
    float32[] stateAZarray
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new controlstate(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.discrepointpersecond !== undefined) {
      resolved.discrepointpersecond = msg.discrepointpersecond;
    }
    else {
      resolved.discrepointpersecond = 0
    }

    if (msg.inicounter !== undefined) {
      resolved.inicounter = msg.inicounter;
    }
    else {
      resolved.inicounter = 0
    }

    if (msg.arraylength !== undefined) {
      resolved.arraylength = msg.arraylength;
    }
    else {
      resolved.arraylength = 0
    }

    if (msg.wall_z !== undefined) {
      resolved.wall_z = msg.wall_z;
    }
    else {
      resolved.wall_z = 0.0
    }

    if (msg.wall_y !== undefined) {
      resolved.wall_y = msg.wall_y;
    }
    else {
      resolved.wall_y = 0.0
    }

    if (msg.parabolictime !== undefined) {
      resolved.parabolictime = msg.parabolictime;
    }
    else {
      resolved.parabolictime = 0.0
    }

    if (msg.thrustarray !== undefined) {
      resolved.thrustarray = msg.thrustarray;
    }
    else {
      resolved.thrustarray = []
    }

    if (msg.tauarray !== undefined) {
      resolved.tauarray = msg.tauarray;
    }
    else {
      resolved.tauarray = []
    }

    if (msg.phiarray !== undefined) {
      resolved.phiarray = msg.phiarray;
    }
    else {
      resolved.phiarray = []
    }

    if (msg.thetaarray !== undefined) {
      resolved.thetaarray = msg.thetaarray;
    }
    else {
      resolved.thetaarray = []
    }

    if (msg.stateXarray !== undefined) {
      resolved.stateXarray = msg.stateXarray;
    }
    else {
      resolved.stateXarray = []
    }

    if (msg.stateYarray !== undefined) {
      resolved.stateYarray = msg.stateYarray;
    }
    else {
      resolved.stateYarray = []
    }

    if (msg.stateZarray !== undefined) {
      resolved.stateZarray = msg.stateZarray;
    }
    else {
      resolved.stateZarray = []
    }

    if (msg.stateVXarray !== undefined) {
      resolved.stateVXarray = msg.stateVXarray;
    }
    else {
      resolved.stateVXarray = []
    }

    if (msg.stateVYarray !== undefined) {
      resolved.stateVYarray = msg.stateVYarray;
    }
    else {
      resolved.stateVYarray = []
    }

    if (msg.stateVZarray !== undefined) {
      resolved.stateVZarray = msg.stateVZarray;
    }
    else {
      resolved.stateVZarray = []
    }

    if (msg.stateAXarray !== undefined) {
      resolved.stateAXarray = msg.stateAXarray;
    }
    else {
      resolved.stateAXarray = []
    }

    if (msg.stateAYarray !== undefined) {
      resolved.stateAYarray = msg.stateAYarray;
    }
    else {
      resolved.stateAYarray = []
    }

    if (msg.stateAZarray !== undefined) {
      resolved.stateAZarray = msg.stateAZarray;
    }
    else {
      resolved.stateAZarray = []
    }

    return resolved;
    }
};

module.exports = controlstate;
