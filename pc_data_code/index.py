import pyaudio

p= pyaudio.PyAudio()
info = p.get_host_api_info_by_index(0)
numberdevices = info.get('deviceCount')

for i in range(0,numberdevices):
    if (p.get_device_info_by_host_api_device_index(0,i).get('maxInputChannels'))>0:
        print("input Device ID",i,"-",p.get_device_info_by_host_api_device_index(0,i).get('name'))