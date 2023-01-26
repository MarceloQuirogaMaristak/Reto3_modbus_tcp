import os
import asyncio
import Adafruit_DHT
import RPi.GPIO as GPIO
from pymodbus.server import StartAsyncTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from threading import Thread

try:

	store = ModbusSlaveContext(
		di = ModbusSequentialDataBlock(0, [1]*100),
		co = ModbusSequentialDataBlock(0, [2]*100),
		hr = ModbusSequentialDataBlock(0, [3]*100),
		ir = ModbusSequentialDataBlock(0, [4]*100))
	context = ModbusServerContext(slaves=store, single=True)

	identity = ModbusDeviceIdentification()
	identity.VendorName  = 'pymodbus'
	identity.ProductCode = 'PM'
	identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
	identity.ProductName = 'pymodbus Server'
	identity.ModelName   = 'pymodbus Server'
	identity.MajorMinorRevision = '1.0'

	def program(context):

		SENSOR_DHT = Adafruit_DHT.DHT11

		dhtGpio = 4
		ledPin = 17

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(ledPin, GPIO.OUT)

		heartbit = 0

		while True:
			humedad, temperatura = Adafruit_DHT.read(SENSOR_DHT, dhtGpio)
			if humedad is not None and temperatura is not None:
				if temperatura >= 20:
						GPIO.output(ledPin,GPIO.HIGH)
				else:
						GPIO.output(ledPin,GPIO.LOW)
				heartbit = heartbit + 1
				os.system('clear')
				print("Temp={0:0.1f}C Hum={1:0.1f}%".format(temperatura, humedad))
				context[1].setValues(3, 0, [heartbit, int(temperatura), int(humedad)])
				print(context[1].getValues(3, 0x00, count=3))

	thread_DHT11 = Thread(target=program, args=(context))
	thread_DHT11.daemon = True
	thread_DHT11.start()

	async def main():
		print("Iniciando Programa")
		await StartAsyncTcpServer(context, identity=identity, address=("192.168.30.5", 502))

	asyncio.run(main())

except KeyboardInterrupt:
	GPIO.cleanup()
	os.system('clear')
