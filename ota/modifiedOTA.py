#!/usr/bin/env python
#
# Original espota.py by Ivan Grokhotkov:
# https://gist.github.com/igrr/d35ab8446922179dc58c
#
# Modified since 2015-09-18 from Pascal Gollor (https://github.com/pgollor)
# Modified since 2015-11-09 from Hristo Gochkov (https://github.com/me-no-dev)
# Modified since 2016-01-03 from Matthew O'Gorman (https://githumb.com/mogorman)
#
# This script will push an OTA update to the ESP
# use it like: python espota.py -i <ESP_IP_address> -I <Host_IP_address> -p <ESP_port> -P <Host_port> [-a password] -f <sketch.bin>
# Or to upload SPIFFS image:
# python espota.py -i <ESP_IP_address> -I <Host_IP_address> -p <ESP_port> -P <HOST_port> [-a password] -s -f <spiffs.bin>
#
# Changes
# 2015-09-18:
# - Add option parser.
# - Add logging.
# - Send command to controller to differ between flashing and transmitting SPIFFS image.
#
# Changes
# 2015-11-09:
# - Added digest authentication
# - Enhanced error tracking and reporting
#
# Changes
# 2016-01-03:
# - Added more options to parser.
#

from __future__ import print_function
import socket
import sys
import os
import optparse
import logging
import hashlib
import random

# Commands
FLASH = 0
SPIFFS = 100
AUTH = 200
PROGRESS = True
TIMEOUT = 10

def serveWithCB(remoteAddr, localAddr, remotePort, localPort, password, filename, command, progressCB, msgCB, errorCB):
  return serve(remoteAddr, localAddr, remotePort, localPort, password, filename, command=command, progressCB=progressCB, msgCB=msgCB, errorCB=errorCB)

def serve(remoteAddr, localAddr, remotePort, localPort, password, filename, command=FLASH, progressCB=None, errorCB=None, msgCB=None):
  localport = 53912
  # Create a TCP/IP socket
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server_address = (localAddr, localPort)
  msg = 'Starting on %s:%s' % (str(server_address[0]), str(server_address[1]))
  if msgCB: msgCB(remoteAddr, msg)
  try:
    sock.bind(server_address)
    sock.listen(1)
  except:
    if errorCB: errorCB(remoteAddr, "Error: Listen Failed")
    return 1

  content_size = os.path.getsize(filename)
  f = open(filename,'rb')
  file_md5 = hashlib.md5(f.read()).hexdigest()
  f.close()
  msg = 'Upload size: %d' % (content_size)
  if msgCB: msgCB(remoteAddr, msg)
  message = '%d %d %d %s\n' % (command, localPort, content_size, file_md5)

  # Wait for a connection
  inv_trys = 0
  data = ''
  msg = 'Sending invitation to %s ' % (remoteAddr)
  if msgCB: msgCB(remoteAddr, msg)
  while (inv_trys < 10):
    inv_trys += 1
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote_address = (remoteAddr, int(remotePort))
    try:
      sent = sock2.sendto(message.encode(), remote_address)
    except:
      sock2.close()
      if errorCB: errorCB(remoteAddr, str('Host ' + str(remoteAddr) + ' Not Found'))
      return 1
    sock2.settimeout(TIMEOUT)
    try:
      data = sock2.recv(37).decode()
      break;
    except:
      sock2.close()
  if (inv_trys == 10):
    if errorCB: errorCB(remoteAddr, 'Error: No response from the ESP')
    return 1
  if (data != "OK"):
    if(data.startswith('AUTH')):
      nonce = data.split()[1]
      cnonce_text = '%s%u%s%s' % (filename, content_size, file_md5, remoteAddr)
      cnonce = hashlib.md5(cnonce_text.encode()).hexdigest()
      passmd5 = hashlib.md5(password.encode()).hexdigest()
      result_text = '%s:%s:%s' % (passmd5 ,nonce, cnonce)
      result = hashlib.md5(result_text.encode()).hexdigest()

      if msgCB: msgCB(remoteAddr, 'Authenticating...')
      message = '%d %s %s\n' % (AUTH, cnonce, result)
      sock2.sendto(message.encode(), remote_address)
      sock2.settimeout(10)
      try:
        data = sock2.recv(32).decode()
      except:
        if errorCB: errorCB(remoteAddr, 'Error: No Answer to our Authentication')
        sock2.close()
        return 1
      if (data != "OK"):
        if errorCB: errorCB(remoteAddr, 'Error: ' + str(data).replace("\r","").replace("\n",""))
        sock2.close()
        sys.exit(1);
        return 1
      if msgCB: msgCB(remoteAddr, 'OK')
    else:
      if errorCB: errorCB(remoteAddr, 'Error Bad Answer: ' + str(data).replace("\r","").replace("\n",""))
      sock2.close()
      return 1
  sock2.close()

  if msgCB: msgCB(remoteAddr, 'Waiting for device...')
  try:
    sock.settimeout(10)
    connection, client_address = sock.accept()
    # chamged this to higher values -> resulted in better behavior
    sock.settimeout(None)
    connection.settimeout(None)
  except:
    if errorCB: errorCB(remoteAddr, 'Error: No response from device')
    sock.close()
    return 1
  try:
    f = open(filename, "rb")
    if (PROGRESS):
      if progressCB: progressCB(remoteAddr, 0)
    else:
      pass
    if msgCB: msgCB(remoteAddr, 'Uploading')
    offset = 0
    while True:
      # chunk = f.read(512)
      # chunk = f.read(1024)
      chunk = f.read(1400)
      # chunk = f.read(1024)
      if not chunk: break
      offset += len(chunk)
      if progressCB: progressCB(remoteAddr, offset/float(content_size))
      connection.settimeout(10)
      try:
        connection.sendall(chunk)
        res = connection.recv(10)
        lastResponseContainedOK = 'OK' in res.decode()
      except:
        if errorCB: errorCB(remoteAddr, 'Error Uploading')
        connection.close()
        f.close()
        sock.close()
        return 1

    if lastResponseContainedOK:
      if msgCB: msgCB(remoteAddr, 'Success')
      connection.close()
      f.close()
      sock.close()
      return 0

    if msgCB: msgCB(remoteAddr, 'Waiting for result...')
    try:
      count = 0
      while True:
        count=count+1
        connection.settimeout(60)
        data = connection.recv(32).decode()
        if msgCB: msgCB(remoteAddr, 'Result: ' + str(data).replace("\r","").replace("\n",""))

        if "OK" in data:
          if msgCB: msgCB(remoteAddr, 'Success')
          connection.close()
          f.close()
          sock.close()
          return 0;
        if count == 5:
          if errorCB: errorCB(remoteAddr, 'Error response from device')
          connection.close()
          f.close()
          sock.close()
          return 1
    except e:
      if errorCB: errorCB(remoteAddr, 'Error: No Result!')
      connection.close()
      f.close()
      sock.close()
      return 1

  finally:
    connection.close()
    f.close()

  sock.close()
  return 1
# end serve



if __name__ == '__main__':
  sys.exit(main(sys.argv))
