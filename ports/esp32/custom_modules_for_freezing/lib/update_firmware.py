import urequests as requests
from esp32 import Partition
import machine

def write_partition(src_url, dest):
    r = requests.get(src_url)
    sz=int(r.headers['Content-Length'])
    if (dest.info()[3]>>12) < ((sz>>12) + 1):
        raise ValueError("Sizes not compatible: {} vs {}".format(sz, dest.info()[3]))
    addr = 0
    blk = bytearray(4096)
    print("Writing ... to {}".format(dest.info()[4]))
    while addr < sz:
        if addr & 0xFFFF == 0:
            print("   ... 0x{:06x}".format(addr))

        r.raw.readinto(blk,len(blk) if sz - addr < 4096 else sz - addr)
        dest.writeblocks(addr >> 12,blk)
        addr += len(blk)
    print("Write OK")

def verify_partition(src_url, dest):
    r = requests.get(src_url)
    sz=int(r.headers['Content-Length'])
    if (dest.info()[3]>>12) < ((sz>>12) + 1):
        raise ValueError("Sizes not compatible: {} vs {}".format(sz, dest.info()[3]))
    addr = 0
    blk = bytearray(4096)
    blk_src = bytearray(4096)
    print("Verifying ...")
    while addr < sz:
        if addr & 0xFFFF == 0:
            print("   ... 0x{:06x}".format(addr))
        r.raw.readinto(blk,len(blk) if sz - addr < 4096 else sz - addr)
        dest.readblocks(addr >> 12,blk_src)
        assert blk_src==blk, "Validation failed"
        addr += len(blk)
    print("Verify OK")
    

def do_update(binary_url="https://github.com/pcr20/uibbq/raw/main/micropython.bin"):
    next_part=Partition(Partition.RUNNING).get_next_update()
    print("Current running {} writing flash for {}".format(Partition(Partition.RUNNING).info()[4],next_part.info()[4]))
    binary_url="https://github.com/pcr20/uibbq/raw/main/micropython.bin"
    write_partition(binary_url, next_part)
    verify_partition(binary_url, next_part)
    print("Partition {} written, booting into it".format(next_part.info()[4]))
    next_part.set_boot()
    machine.reset()

