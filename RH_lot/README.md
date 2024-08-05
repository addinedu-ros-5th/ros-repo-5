# Lot_check 

## manual

- before RFID tag 
![image](https://github.com/user-attachments/assets/c06d68d5-f87c-44b5-bf57-b8bd13bd174e)

- after RFID tag
![image](https://github.com/user-attachments/assets/a4d3043b-28ca-49ee-aba3-13e724595f43)


### Output signal : PXX000 (XX : parking area) - parking complete!


***

## error: ordered comparison of pointer with integer zero ('byte' {aka 'unsigned char'} and 'int')

```
/home/tspoon/workspace/libraries/MFRC522/src/MFRC522Extended.cpp: In member function 'MFRC522::StatusCode MFRC522Extended::TCL_Transceive(TagInfo*, byte*, byte, byte*, byte*)':
/home/tspoon/workspace/libraries/MFRC522/src/MFRC522Extended.cpp:824:34: error: ordered comparison of pointer with integer zero ('byte*' {aka 'unsigned char*'} and 'int')
  824 |         if (backData && (backLen > 0)) {
      |                          ~~~~~~~~^~~
/home/tspoon/workspace/libraries/MFRC522/src/MFRC522Extended.cpp:847:42: error: ordered comparison of pointer with integer zero ('byte*' {aka 'unsigned char*'} and 'int')
  847 |                 if (backData && (backLen > 0)) {
      |                                  ~~~~~~~~^~~

exit status 1

Compilation error: exit status 1
```

### solution
- libraries/MFRC522/src/MFRC522Extended.cpp 변경

```
// 수정된 부분:
if (backData != nullptr && *backLen > 0) {
    if (*backLen < in.inf.size)
        return STATUS_NO_ROOM;

    *backLen = in.inf.size;
    memcpy(backData, in.inf.data, in.inf.size);
}

// ... (중간 코드 생략) ...

// 수정된 부분:
if (backData != nullptr && *backLen > 0) {
    if ((*backLen + ackDataSize) > totalBackLen)
        return STATUS_NO_ROOM;ㅔ

    memcpy(&(backData[*backLen]), ackData, ackDataSize);
    *backLen += ackDataSize;
}
```


## Failed uploading: uploading error: exit status 2

```
Sketch uses 290057 bytes (22%) of program storage space. Maximum is 1310720 bytes.
Global variables use 20312 bytes (6%) of dynamic memory, leaving 307368 bytes for local variables. Maximum is 327680 bytes.
esptool.py v4.6
Serial port /dev/ttyACM0

A fatal error occurred: Could not open /dev/ttyACM0, the port doesn't exist
Failed uploading: uploading error: exit status 2
```

### solution
```
sudo chmod a+rw /dev/ttyACM0
```
