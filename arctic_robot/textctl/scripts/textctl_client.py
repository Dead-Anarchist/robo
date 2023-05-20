#!/usr/bin/env python
import httplib

# connect to server
conn = httplib.HTTPConnection('127.0.0.1', 23009)
print('Connection established.')

inp = ''
while inp != 'quit':
    inp = raw_input('')
    if inp == 'quit':
        break
    conn.request('POST', '', inp)
    r1 = conn.getresponse()
    responce = r1.read()
    print('>>>'+ str(r1.status)+' '+str(responce))
