#!/usr/bin/python3
# Broadcasts connectable advertising packets

from march_ble_ipd import bluetooth_constants, bluetooth_exceptions, bluetooth_gatt
import dbus
import dbus.exceptions
import dbus.service
import dbus.mainloop.glib
import sys
import random
from gi.repository import GLib
sys.path.insert(0, '.')

bus = None
adapter_path = None
adv_mgr_interface = None
connected = 0

# much of this code was copied or inspired by test\example-advertisement in the BlueZ source
class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/ldsg/advertisement'
    def __init__(self, bus, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = advertising_type
        self.service_uuids = None
        self.manufacturer_data = None
        self.solicit_uuids = None
        self.service_data = None
        self.local_name = 'Hello'
        self.include_tx_power = False
        self.data = None
        self.discoverable = True
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        properties = dict()
        properties['Type'] = self.ad_type

        if self.service_uuids is not None:
            properties['ServiceUUIDs'] = dbus.Array(self.service_uuids, signature='s')

        if self.solicit_uuids is not None:
            properties['SolicitUUIDs'] = dbus.Array(self.solicit_uuids, signature='s')

        if self.manufacturer_data is not None:
            properties['ManufacturerData'] = dbus.Dictionary(self.manufacturer_data, signature='qv')

        if self.service_data is not None:
            properties['ServiceData'] = dbus.Dictionary(self.service_data,
        signature='sv')
            
        if self.local_name is not None:
            properties['LocalName'] = dbus.String(self.local_name)

        if self.discoverable is not None and self.discoverable == True:
            properties['Discoverable'] = dbus.Boolean(self.discoverable)

        if self.include_tx_power:
            properties['Includes'] = dbus.Array(["tx-power"], signature='s')

        if self.data is not None:
            properties['Data'] = dbus.Dictionary(self.data, signature='yv')

        print(properties)
        return {bluetooth_constants.ADVERTISING_MANAGER_INTERFACE:properties}
    
    def get_path(self):
        return dbus.ObjectPath(self.path)
    
    @dbus.service.method(bluetooth_constants.DBUS_PROPERTIES, in_signature='s',out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != bluetooth_constants.ADVERTISEMENT_INTERFACE:
            raise bluetooth_exceptions.InvalidArgsException()
        return self.get_properties()[bluetooth_constants.ADVERTISING_MANAGER_INTERFACE]
    
    @dbus.service.method(bluetooth_constants.ADVERTISING_MANAGER_INTERFACE, in_signature='', out_signature='')
    def Release(self):
        print('%s: Released' % self.path)

class TemperatureCharacteristic(bluetooth_gatt.Characteristic):
    temperature = 0
    delta = 0
    notifying = False
    callback = None

    def __init__(self, bus, index, service, callback=None):
        bluetooth_gatt.Characteristic.__init__(self, bus, index,
                                                bluetooth_constants.TEMPERATURE_CHR_UUID,
                                                ['write', 'read'],
                                                service)
        self.notifying = True
        self.temperature = random.randint(0, 50)
        print("Initial temperature set to "+str(self.temperature))
        self.delta = 0
        self.callback = callback
        # GLib.timeout_add(1000, self.simulate_temperature)

    def ReadValue(self, options):
        print('ReadValue in TemperatureCharacteristic called')
        print('Returning '+str(self.temperature))
        value = []
        value.append(dbus.UInt16(self.temperature))
        return value
    
    def WriteValue(self, value, options):
        print('WriteValue in TemperatureCharacteristic called')
        print(int(value[0]))
        self.temperature = int(value[0])  # interpret the first byte as an integer
        print('New temperature is '+str(self.temperature))
        # send a write response back to the client
        self.PropertiesChanged(bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE, {'Value': [dbus.Byte(self.temperature)]}, [])

        print("callback="+str(self.callback))
        if self.callback:
            print("entering correct if statement")
            try:
                self.callback(self.temperature)
            except Exception as e:
                print(f"Error occurred while calling callback: {e}")
            print("calling callback with temperature="+str(self.temperature)+"C")
    
    def simulate_temperature(self):
        self.delta = random.randint(-1, 1)
        self.temperature = self.temperature + self.delta
        if (self.temperature > 50):
            self.temperature = 50
        elif (self.temperature < 0):
            self.temperature = 0
        print("simulated temperature: "+str(self.temperature)+"C")
        if self.notifying:  # Only notify if notifications are enabled
            self.notify_temperature()
        GLib.timeout_add(1000, self.simulate_temperature)

    def notify_temperature(self):
        value = []
        value.append(dbus.Byte(self.temperature))
        print("notifying temperature="+str(self.temperature))
        self.PropertiesChanged(bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE, { 'Value': value }, [])
        return self.notifying
    
    def StartNotify(self):
        print("starting notifications")
        self.notifying = True

    def StopNotify(self):
        print("stopping notifications")
        self.notifying = False

class TemperatureService(bluetooth_gatt.Service):
    # Fake micro:bit temperature service that simulates temperature sensor measurements
    #ref: https://lancaster-university.github.io/microbit-docs/resources/bluetooth/bluetooth_profile.html
    #temperature_period characteristic not implemented to keep things simple
    def __init__(self, bus, path_base, index, callback=None):
        print("Initialising TemperatureService object")
        bluetooth_gatt.Service.__init__(self, bus, path_base, index,
                                        bluetooth_constants.TEMPERATURE_SVC_UUID, True)
        print("Adding TemperatureCharacteristic to the service")
        self.add_characteristic(TemperatureCharacteristic(bus, 0, self, callback))

class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus, callback=None):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        print("Adding TemperatureService to the Application")
        self.add_service(TemperatureService(bus, '/org/bluez/ldsg', 0, callback))
    
    def get_path(self):
        return dbus.ObjectPath(self.path)
    
    def add_service(self, service):
        self.services.append(service)

    def get_service(self, index):
        if index < len(self.services):
            return self.services[index]
        else:
            raise IndexError("Invalid service index")

    @dbus.service.method(bluetooth_constants.DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        print('GetManagedObjects')

        for service in self.services:
            print("GetManagedObjects: service="+service.get_path())
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()
        return response


class BluetoothServer:
    def __init__(self, callback=None):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        # we're assuming the adapter supports advertising
        adapter_path = bluetooth_constants.BLUEZ_NAMESPACE + bluetooth_constants.ADAPTER_NAME
        print(adapter_path)

        bus.add_signal_receiver(self.properties_changed,
                                dbus_interface = bluetooth_constants.DBUS_PROPERTIES,
                                signal_name = "PropertiesChanged",
                                path_keyword = "path")

        bus.add_signal_receiver(self.interfaces_added,
                                dbus_interface = bluetooth_constants.DBUS_OM_IFACE,
                                signal_name = "InterfacesAdded")

        self.adv_mgr_interface = dbus.Interface(bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME,adapter_path), bluetooth_constants.ADVERTISING_MANAGER_INTERFACE)

        # we're only registering one advertisement object so index (arg2) is hard coded as 0
        self.adv = Advertisement(bus, 0, 'peripheral')
        self.start_advertising()
        

        print("Advertising as "+self.adv.local_name)

        app = Application(bus, callback)

        print('Registering GATT application...')

        service_manager = dbus.Interface(bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME,
                                                        adapter_path),
                                        bluetooth_constants.GATT_MANAGER_INTERFACE)
        service_manager.RegisterApplication(app.get_path(), {},
                                            reply_handler=self.register_app_cb,
                                            error_handler=self.register_app_error_cb)

        self.mainloop = GLib.MainLoop()
        self.mainloop.run()

    def register_app_error_cb(self, error):
            print('Failed to register application: ' + str(error))
            self.mainloop.quit()

    
    def register_ad_cb(self):
        print('Advertisement registered OK')

    def register_ad_error_cb(self, error):
        print('Error: Failed to register advertisement: ' + str(error))
        self.mainloop.quit()

    def set_connected_status(self, status):
        global connected
        if (status == 1):
            print("connected")
            connected = 1
            self.stop_advertising()
        else:
            print("disconnected")
            connected = 0
            self.start_advertising()

    def properties_changed(self, interface, changed, invalidated, path):
        if (interface == bluetooth_constants.DEVICE_INTERFACE):
            if ("Connected" in changed):
                self.set_connected_status(changed["Connected"])

    def interfaces_added(self, path, interfaces):
        if bluetooth_constants.DEVICE_INTERFACE in interfaces:
            properties = interfaces[bluetooth_constants.DEVICE_INTERFACE]
            if ("Connected" in properties):
                self.set_connected_status(properties["Connected"])

    def stop_advertising(self):
        print("Unregistering advertisement", self.adv.get_path())
        self.adv_mgr_interface.UnregisterAdvertisement(self.adv.get_path())

    def start_advertising(self):

        # we're only registering one advertisement object so index (arg2) is hard coded as 0
        print("Registering advertisement", self.adv.get_path())
        self.adv_mgr_interface.RegisterAdvertisement(self.adv.get_path(), {},
                                                reply_handler=self.register_ad_cb,
                                                error_handler=self.register_ad_error_cb)
        
    def register_app_cb(self):
        print('GATT application registered')

