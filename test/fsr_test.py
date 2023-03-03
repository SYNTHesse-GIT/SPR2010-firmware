from matplotlib.cbook import ls_mapper
from dynamixel_sdk import PacketHandler, PortHandler

port = PortHandler('/dev/tty.usbserial-AL01QHDB')
port.openPort()
port.setBaudRate(1000000)

ph = PacketHandler(2.0)

result = ph.ping(port, 100)
print([hex(item) for item in result])

