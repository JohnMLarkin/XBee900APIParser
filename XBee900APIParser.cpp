#include <mbed.h>
#include <rtos.h>
#include <XBee900APIParser.h>

XBee900APIParser::XBee900APIParser(PinName tx, PinName rx, int baud) : _modem(tx,rx,baud) {
  // for (int i = 0; i < MAX_INCOMING_FRAMES; i++) {
  //   _inFramesLengths[i] = 0;
  // }
  // _numInFrames = 0;
  // _inBufferLength = -1;
  for (int i = 0; i < MAX_INCOMING_FRAMES; i++) {
    _frameBuffer.frame[i].type = 0xFF;
    _frameBuffer.frame[i].id = 0x00;
    _frameBuffer.frame[i].length = 0;
  }
  _frameBuffer.length = 0;
  _partialFrame.status = 0x00;
  _time_out = 1000;
  _isAssociated = false;
  _failedTransmits = 0;
  _maxFailedTransmits = 5;
  _frameAlertThreadId = NULL;
  _updateBufferThread.start(callback(this, &XBee900APIParser::_move_frame_to_buffer));
  _modem.attach(callback(this,&XBee900APIParser::_pull_byte),SerialBase::RxIrq);
}

bool XBee900APIParser::associated() {
  if (!_isAssociated) {
    _verify_association();
  }
  return _isAssociated;
}

/** find_frame
 *   Returns frame with specified frame type and frame ID. The frame is also
 *     removed from the buffer.
 * 
 *   @returns true if match was found
 */
bool XBee900APIParser::find_frame(char frameType, char frameID, apiFrame_t* frame) {
  if (_frameBufferMutex.trylock_for(_time_out)) {
    if (_frameBuffer.length>0) {
      for (int i = 0; i < _frameBuffer.length; i++) {
        if ((_frameBuffer.frame[i].type == frameType) && (_frameBuffer.frame[i].id == frameID)) {
          frame->type = frameType;
          frame->id = frameID;
          frame->length = _frameBuffer.frame[i].length;
          for (int j = 0; j < frame->length; j++)
            frame->data[j] = _frameBuffer.frame[i].data[j];
          _frameBufferMutex.unlock();
          _remove_frame_by_index(i);
          return true;
        }
      }
    }
    _frameBufferMutex.unlock();
  }
  return false;
}

/** find_frame
 *   Returns frame with specified frame type. The frame is also
 *     removed from the buffer.
 * 
 *   @returns true if match was found
 */
bool XBee900APIParser::find_frame(char frameType, apiFrame_t* frame) {
  return find_frame(frameType, 0xFF, frame);
}

void XBee900APIParser::flush_old_frames(char frameType, char frameID) {
  apiFrame_t frame;
  while (find_frame(frameType, frameID, &frame));
}

uint64_t XBee900APIParser::get_address(string ni) {
  uint64_t address;
  Timer t;
  char frameID;
  char status;
  apiFrame_t frame;
  int len;
  bool foundFrame;
  _make_AT_frame("DN", ni, &frame);
  flush_old_frames(frame.type, frame.id);
  frameID = frame.id;
  send(&frame);
  wait_ms(5);
  t.start();
  foundFrame = false;
  while ((t.read_ms()<10*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (!foundFrame) printf("Timed out after DN!\r\n");
  if ((!foundFrame) || (frame.length != 3)) return 0;
  if (!((frame.data[0] = 'D') && (frame.data[1] == 'N') && (frame.data[2]==0))) return 1;
  _make_AT_frame("DH", &frame);
  flush_old_frames(frame.type, frame.id);
  frameID = frame.id;
  send(&frame);
  t.reset();
  foundFrame = false;
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (!foundFrame) printf("Timed out after DH!\r\n");
  if ((!foundFrame) || (frame.length != 7)) return 0;
  address = 0;
  for (int i = 0; i < 4; i++) {
    address = (address << 8) | frame.data[3+i];
  }
  _make_AT_frame("DL", &frame);
  flush_old_frames(frame.type, frame.id);
  frameID = frame.id;
  send(&frame);
  t.reset();
  foundFrame = false;
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (!foundFrame) printf("Timed out after DL!\r\n");
  if ((!foundFrame) || (frame.length != 7)) return 0;
  for (int i = 0; i < 4; i++) {
    address = (address << 8) | frame.data[3+i];
  }
  return address;
}

char XBee900APIParser::last_RSSI() {
  Timer t;
  char frameID;
  char rssi = 0xFF;
  apiFrame_t frame;
  bool foundFrame;
  _make_AT_frame("DB", &frame);
  frameID = frame.id;
  send(&frame);
  t.start();
  foundFrame = false;
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (!foundFrame) return 0xFF;
  if (frame.length == 6) {
    if ((frame.data[2]=='D') && (frame.data[3]=='B')&& (frame.data[4]==0)) {
      rssi = frame.data[5];
    }
  }
  return rssi;
}

bool XBee900APIParser::readable() {
  bool hasFrames = false;
  if (_frameBufferMutex.trylock_for(_time_out)) {
    hasFrames = _frameBuffer.length > 0;
    _frameBufferMutex.unlock();
  }
  return hasFrames;
}

bool XBee900APIParser::get_oldest_frame(apiFrame_t* frame) {
  if (_frameBufferMutex.trylock_for(_time_out)) {
    if (_frameBuffer.length > 0) {
      frame->type = _frameBuffer.frame[0].type;
      frame->id = _frameBuffer.frame[0].id;
      frame->length = _frameBuffer.frame[0].length;
      for (int i = 0; i < frame->length; i++)
        frame->data[i] = _frameBuffer.frame[0].data[i];
      _frameBufferMutex.unlock();
      _remove_frame_by_index(0);
      return true;
    } 
    _frameBufferMutex.unlock();
  }
  return false;
}

int XBee900APIParser::rxPacket(char* payload, uint64_t* address) {
  apiFrame_t frame;
  bool foundFrame;
  uint64_t who = 0;
  foundFrame = find_frame(0x90, &frame);
  if (foundFrame) {
    for (int i = 0; i < 8; i++) {
      who = (who << 8) | frame.data[i];
    }
    *address = who;
    for (int i = 0; i < (frame.length-9); i++) {
      payload[i] = frame.data[i+11];
    }
    return (frame.length-11);
  } else return 0;
}

bool XBee900APIParser::send(apiFrame_t* frame) {
  bool success = true;
  uint32_t checksum = frame->type;
  checksum = checksum + frame->id;
  for (int i = 0; i < frame->length; i++) {
    checksum = checksum + frame->data[i];
  }
  checksum = checksum & 0xFF;
  checksum = 0xFF - checksum;
  Timer t;
  t.start();
  if (_modemTxMutex.trylock_for(_time_out)) {
    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc(0x7E);
    } else success = false; // Timed out

    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc((frame->length+2) >> 8);
    } else success = false; // Timed out
    
    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc((frame->length+2) & 0xFF);
    } else success = false; // Timed out
    
    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc(frame->type);
    } else success = false; // Timed out

    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc(frame->id);
    } else success = false; // Timed out

    for (int i = 0; i < frame->length; i++) {
      while ((t.read() < _time_out) && (!_modem.writeable())) {}
      if (_modem.writeable() && success) {
        _modem.putc(frame->data[i]);
      } else success = false; // Timed out
    }

    while ((t.read() < _time_out) && (!_modem.writeable())) {}
    if (_modem.writeable() && success) {
      _modem.putc(checksum);
    } else success = false; // Timed out
    _modemTxMutex.unlock(); 
  }
  return success;
}

void XBee900APIParser::set_frame_alert_thread_id(osThreadId_t threadID) {
  _frameAlertThreadId = threadID;
}

void XBee900APIParser::set_max_failed_transmits(int maxFails) {
  if ((maxFails>0) && (maxFails<20)) _maxFailedTransmits = maxFails;
}

void XBee900APIParser::set_timeout(int time_ms) {
  if ((time_ms > 0) && (time_ms < 5000)) _time_out = time_ms;
}

int XBee900APIParser::txAddressed(uint64_t address, char* payload, int len) {
  if (len>(MAX_FRAME_LENGTH)) return -1;
  apiFrame_t frame;
  bool foundFrame;
  char frameID = 0x00;
  for (int i = 0; i < len; i++) {
    frameID = frameID + payload[i];
  }
  frame.type = 0x00; // Tx Request Frame Type
  frame.id = frameID;
  for (int i = 0; i < 8; i++) {
    frame.data[i] = (address >> ((7-i)*8)) & 0xFF;
  }
  frame.data[8] = 0x00;  // No options
  for (int i = 0; i < len; i++) {
    frame.data[9+i] = payload[i];
  }
  frame.length = len + 9;
  send(&frame);
  Timer t;
  t.start();
  foundFrame = false;
  wait_ms(7);
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x89, frameID, &frame);
    if (!foundFrame) wait_ms(7);
  }
  if (foundFrame) {
    if (frame.data[0] == 0x00) {
      _failedTransmits = 0;
      return 0;
    } else {
      _failedTransmits++;
      if (_failedTransmits>= _maxFailedTransmits) {
        _disassociate();
        _failedTransmits = 0;
        return -2;
      }
    }
  }
  return -3;
}

int XBee900APIParser::txBroadcast(char* payload, int len) {
  return txAddressed(0xFFFF, payload, len);
}

void XBee900APIParser::_disassociate() {
  Timer t;
  apiFrame_t frame;
  bool foundFrame;
  _make_AT_frame("DA", &frame);
  char frameID = frame.id;
  send(&frame);
  t.start();
  foundFrame = false;
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (foundFrame) {
    if ((frame.data[0]=='D') && (frame.data[1]=='A') && (frame.data[2]==0)) {
      _isAssociated = false;
    }
  }
}

void XBee900APIParser::_make_AT_frame(string cmd, apiFrame_t* frame) {
  _make_AT_frame(cmd, "", frame);
}

void XBee900APIParser::_make_AT_frame(string cmd, string param, apiFrame_t* frame) {
  frame->type = 0x08; // Local AT command frame
  if (cmd.length()==2) {
    frame->data[0] = cmd[0];
    frame->data[1] = cmd[1];
  }
  frame->id = cmd[0] + cmd[1];
  if (param.length()>0) {
    for (int i = 0; i < param.length(); i++)
      frame->data[2+i] = param[i];
  }
  frame->length = 2 + param.length();
}

void XBee900APIParser::_pull_byte() {
  char buff;
  uint16_t len;
  uint32_t checksum = 0;
  while (_modem.readable() && (_partialFrame.status < 0x06)) {
    buff = _modem.getc();
    switch (_partialFrame.status) {
      case 0x00:  // Waiting for start of new frame
        if (buff == 0x7E) {  // Frame start byte should be 0x7E
          _partialFrame.status = 0x01;
        }
        break;
      case 0x01:  // First frame length byte
        _partialFrame.frame.length = buff << 8;
        _partialFrame.status = 0x02;
        break;
      case 0x02: // Second frame length byte
        _partialFrame.frame.length = (_partialFrame.frame.length | buff)-2;
        _partialFrame.rcvd = 0;
        _partialFrame.status = 0x03;
        break;
      case 0x03: // Frame type
        _partialFrame.frame.type = buff;
        switch (buff) { // frame types with FrameID
          case 0x00: _partialFrame.status = 0x04; break;
          case 0x08: _partialFrame.status = 0x04; break;
          case 0x17: _partialFrame.status = 0x04; break;
          case 0x88: _partialFrame.status = 0x04; break;
          case 0x89: _partialFrame.status = 0x04; break;
          case 0x97: _partialFrame.status = 0x04; break;
          default: // No Frame ID for this type
            _partialFrame.frame.id = 0xFF;
            _partialFrame.frame.length++;
            _partialFrame.status = 0x05;
        }
        break;
      case 0x04: // Frame ID
        _partialFrame.frame.id = buff;
        _partialFrame.status = 0x05;
        break;
      case 0x05:
        len = _partialFrame.frame.length;
        if (len > MAX_FRAME_LENGTH) { // Incoming frame won't fit!
          _partialFrame.status = 0x00;
        } else if (_partialFrame.rcvd < len) {  // Waiting for rest of frame
          _partialFrame.frame.data[_partialFrame.rcvd] = buff;
          _partialFrame.rcvd++;
        } else { // This should be the checksum
          switch (_partialFrame.frame.type) { // frame types with FrameID
            case 0x00: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            case 0x08: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            case 0x17: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            case 0x88: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            case 0x89: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            case 0x97: checksum = _partialFrame.frame.type + _partialFrame.frame.id; break;
            default: // No Frame ID for this type
              checksum = _partialFrame.frame.type;
          }
          for (int i = 0; i < len; i++)
            checksum = checksum + _partialFrame.frame.data[i];
          checksum = (checksum + buff) & 0xFF;
          if (checksum == 0xFF) { // Frame is good!  Save to buffer.
            if (_partialFrame.frame.type == 0x8A) { // Intercept modem status frames
              switch (_partialFrame.frame.data[0]) {
                case 0x02:
                  _isAssociated = true;
                  _failedTransmits = 0;
                  break;
                case 0x06:
                  _isAssociated = true;
                  _failedTransmits = 0;
                  break;
                default:
                  _isAssociated = false;
              }
              _partialFrame.status = 0x00;
            } else {
              _modem.attach(NULL); // Stop interrupts on serial input
              _partialFrame.status = 0x06; 
              osSignalSet(_updateBufferThreadId, 0x06); // Trigger copy to frame buffer outside of ISR
            }
          } else { // Checksum doesn't match.  Bad frame!
            _partialFrame.status = 0x00; // There should be some error signaling
          }
        }
        break;
      case 0x06:
        break;
    }
  }
}

void XBee900APIParser::_move_frame_to_buffer() {
  _updateBufferThreadId = osThreadGetId();

  while (true) {
    osSignalWait(0x06, osWaitForever);
    if (_frameBufferMutex.trylock_for(5*_time_out)) {
      if (_frameBuffer.length == MAX_INCOMING_FRAMES) {  // Buffer full, drop oldest frame
        _remove_frame_by_index(0);
      }
      int n = _frameBuffer.length; // Save current length for ease of copying to buffer
      _frameBuffer.frame[n].id = _partialFrame.frame.id;
      _frameBuffer.frame[n].type = _partialFrame.frame.type;
      _frameBuffer.frame[n].length = _partialFrame.frame.length;
      for (int i = 0; i < _partialFrame.frame.length; i++)
        _frameBuffer.frame[n].data[i] = _partialFrame.frame.data[i];
      _frameBuffer.length++;
      _frameBufferMutex.unlock();
      _partialFrame.status = 0x00;
      if (_frameAlertThreadId) osSignalSet(_frameAlertThreadId, 0x01); 
      _modem.attach(callback(this,&XBee900APIParser::_pull_byte),SerialBase::RxIrq);
    }
  }
}

void XBee900APIParser::_remove_frame_by_index(int n) {
  if (n< _frameBuffer.length) {
    for (int i = n + 1; i < _frameBuffer.length; i++) {
      for (int j = 0; j < _frameBuffer.frame[i].length; j++) {
        _frameBuffer.frame[i-1].data[j] = _frameBuffer.frame[i].data[j];
      }
      _frameBuffer.frame[i-1].type = _frameBuffer.frame[i].type;
      _frameBuffer.frame[i-1].id = _frameBuffer.frame[i].id;
      _frameBuffer.frame[i-1].length = _frameBuffer.frame[i].length;
    }
  }
  if (_frameBuffer.length>0) _frameBuffer.length--;
}

void XBee900APIParser::_verify_association() {
  Timer t;
  apiFrame_t frame;
  bool foundFrame;
  char status = 0xFE;
  _make_AT_frame("AI", &frame);
  char frameID = frame.id;
  _isAssociated = false;
  foundFrame = false;
  send(&frame);
  t.start();
  while ((t.read_ms()<2*_time_out) && (!foundFrame)) {
    foundFrame = find_frame(0x88, frameID, &frame);
    if (!foundFrame) wait_ms(5);
  }
  if (foundFrame) {
    if ((frame.data[0]=='A') && (frame.data[1]=='I') && (frame.data[2]==0)) {
      status = frame.data[3];
    }
  }
  if (status == 0x00) _isAssociated = true;
}



