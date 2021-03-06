# depthai-spi-api
## SPI Protocol

SPI messaging is currently arranged in 2 layers. The first is the spi protocol. The spi protocol is the lowest level. It defines a standard packet for all SPI communication. It is a 256 byte packet arranged in the following manner:

```
typedef struct {
    uint8_t start;
    uint8_t data[SPI_PROTOCOL_PAYLOAD_SIZE];
    uint8_t crc[2];
    uint8_t end;
} SpiProtocolPacket;
```
start and end are constant bytes to mark the beginning and end of packets.
```
static const uint8_t START_BYTE_MAGIC = 0b10101010;
static const uint8_t END_BYTE_MAGIC = 0b00000000;
```
## SPI Messaging
On top of this we have a layer called SPI messaging. This code defines the following:
* A list of a supported commands,
* A way to encapsulate commands going to the MyriadX over SPI.
* A way to receive and parse command responses. 

## SPI API
Finally, on top of the messaging layer, we have a SPI API to make basic useage more straightforward. This layer will manage most of the things necessary for sending and receiving SPI messages.

An example of current capabilites can be found at this repo:
https://github.com/luxonis/esp32-spi-message-demo/tree/gen2_common_objdet

NOTE: This is still in flux so there may be potential changes to the API.
