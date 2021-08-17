import os

class DeleteData:
	def __init__(self):
		pass
	def delete(self):
		try:
			os.chdir('/sd/ControllerData/ControllerDataSubFolder/Ottobock')
			tmp = os.listdir()
			for x in tmp:
				try:
					os.remove(x)
				except:
					pass
			os.chdir('/sd/ControllerData/ControllerDataSubFolder')
			os.rmdir('Ottobock')
		except:
			pass

		try:
			os.chdir('/sd/ControllerData/ControllerDataSubFolder/Psyonic')
			tmp = os.listdir()
			for x in tmp:
				try:
					os.remove(x)
				except:
					pass
			os.chdir('/sd/ControllerData/ControllerDataSubFolder')
			os.rmdir('Psyonic')
		except:
			pass

		try:
			os.chdir('/sd/ControllerData/ControllerDataSubFolder')
			tmp = os.listdir()
			for x in tmp:
				try:
					os.remove(x)
				except:
					pass
				try:
					os.rmdir(x)
				except:
					pass
		except:
			pass

		os.chdir('/sd/ControllerData')
		try:
			os.rmdir('ControllerDataSubFolder')
		except:
			pass
		try:
			tmp = os.listdir()
			for x in tmp:
				os.remove(x)
		except:
			pass

		os.chdir('/sd')
		try:
			os.rmdir('ControllerData')
		except:
			pass