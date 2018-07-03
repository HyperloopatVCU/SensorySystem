Modified Arduino Ethernet Library
==========

Modified version of the [Arduino](http://arduino.cc) Ethernet v1.1.1 library.
- Added functions:
  - `EthernetClient.remoteIP()`
  - `EthernetClient.remotePort()`
  - `EthernetClient.localPort()`
  - `EthernetClient.setClientTimeout()`
- Removed the delay from `Ethernet.begin()`.

<a id="installation"></a>
#### Installation
- This library is only compatible with Arduino versions 1.5 and up. If you are using Arduino IDE 1.0.x then select the [Arduino-IDE-1.0.x branch](https://github.com/per1234/EthernetMod/tree/Arduino-IDE-1.0.x) instead.
- Download the most recent version of the modified Ethernet library here: https://github.com/per1234/EthernetMod/archive/Arduino-IDE-1.5plus.zip
- Extract the downloaded file
- Rename the extracted folder Ethernet
- Copy the Ethernet folder to the libraries folder under your sketchbook folder
- Reopen the Arduino IDE


<a id="usage"></a>
#### Usage
##### `EthernetClient.remoteIP()`
See **File > Examples > EthenetMod > ChatServer** for demonstration of using this function. Thanks to [ntruchsess](https://github.com/ntruchsess/Arduino-1/commit/ca37de4ba4ecbdb941f14ac1fe7dd40f3008af75).
- Returns: The IP address of the remote connection.
  - Type: IPAddress

##### `EthernetClient.remotePort()`
See **File > Examples > EthenetMod > ChatServer** for demonstration of using this function. Thanks to [ntruchsess](https://github.com/ntruchsess/Arduino-1/commit/ca37de4ba4ecbdb941f14ac1fe7dd40f3008af75).
- Returns: The port of the remote connection.
  - Type: unsigned int

##### `EthernetClient.localPort()`
See **File > Examples > EthenetMod > ChatServer** for demonstration of using this function. Thanks to [ntruchsess](https://github.com/ntruchsess/Arduino-1/commit/937bce1a0bb2567f6d03b15df79525569377dabd).
- Returns: The local port the client is connected to.
  - Type: unsigned int

##### `EthernetClient.setClientTimeout(timeout)`
Set the timeout duration for `EthernetClient.connect()` and `EthernetClient.stop()`. The initial value is 1000ms.
- Parameter: **timeout** - (ms)The timeout duration value.
  - Type: unsigned int
- Returns: none

I have removed the `delay(300)` from `Ethernet.begin()` so you may need to add a delay in your sketch if it is not reliably initializing the Ethernet controller.

For all the standard functions included in the library see the official Arduino Ethernet documentation: http://arduino.cc/en/reference/ethernet
The `read()` function in the Client class inherits from Stream so the Stream functions are also available, see the Arduino Stream documentation: http://arduino.cc/en/Reference/Stream


<a id="timeouts"></a>
#### Timeouts
The key to the most responsive, reliable Ethernet communication is setting appropriate timeout values. The default values are very conservative. Setting the timeout values too high can lead to long blocking delays if an Ethernet process is unsuccessful. If you have a watchdog timeout of less than the timeout duration then it may cause your Arduino to reset. Setting the timeout values too low can cause frequent communication failures. The optimal values are different for each network configuration so you will need to do some experimentation to find them.

- `EthernetClient.setClientTimeout()`: see information in the **Usage** section above.
- `EthernetClient.setTimeout()`: (ms)Timeout for all Stream functions. The initial value is 1000ms.
- W5x00 configuration: You must add `#include <utility/w5100.h>` after the line `#include <Ethernet.h>` in your code to make these functions available.
  - `W5100.setRetransmissionTime()`: (0.1ms)Set the timeout for the W5x00 module. The initial value is 200ms.
  - `W5100.setRetransmissionCount()`: Retransmission Count. 1 is the minimum value. The initial value is 8. 8 retries with 200ms timeout means the W5x00 chip will take 1600ms to timeout with the initial values.


<a id="license"></a>
#### License
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
