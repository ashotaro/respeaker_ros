#!/usr/local/bin/python

import usb


class UsbDevice(object):
    def __init__(self, vendor_id=None, product_id=None):
	# busses = usb.busses()
        devlist = tuple(usb.core.find(find_all=True, idVendor=vendor_id, idProduct=product_id))
        for i in range(len(devlist)):
            print 'Bus : %d, Address : %d' % (devlist[i].bus, devlist[i].address)
       
        # devlist[0].set_configuration()
        # devlist[1].set_configuration()

    # def open(self):
	# if hasattr(self, 'handle'):
	    # raise RuntimeError, 'Device already opened'
	# self.handle = self.device.open()
	# product = self.handle.getString(self.device.iProduct, 20)
	# manufacturer = self.handle.getString(self.device.iManufacturer, 20)
	# bus = self.handle.getString(self.device.bus, 20)
	# address = self.handle.getString(self.device.address, 20)
	# print product, manufacturer
        # print bus, #address

    # def close(self):
	# if hasattr(self, 'handle'):
	    # self.handle.releaseInterface()
	    # del self.handle
	# else:
	    # raise RuntimeError, 'Device not opened'


def main():
    dev = UsbDevice(0x2886, 0x0018)
    # dev.open()
#    dev.close()

if __name__ == '__main__':
    main()
