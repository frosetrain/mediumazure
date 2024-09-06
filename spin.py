"""spin."""

from pybricks.hubs import PrimeHub
from ustruct import unpack_from

hub = PrimeHub()

# buf = bytearray(6 * 2)  # 2 bytes for each unsigned short
buf = hub.system.storage(0, read=6 * 2)
for i in range(6):
    print(unpack_from("H", buf, i * 2))
