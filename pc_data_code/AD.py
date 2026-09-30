import pyaudio
import wave
import time
import sys
import signal
import os

RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6      # change to 1 if you only need mono
RESPEAKER_WIDTH = 2         # 16-bit = 2 bytes
CHUNK = 1024
WAVE_OUTPUT_FILENAME = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test5.wav")
DEVICE_NAME_KEYWORD = "ReSpeaker"

# Global stop flag
stop_requested = False

def handle_stop_signal(signum, frame):
    global stop_requested
    print(f"\nStop signal received ({signum}). Finalizing and saving audio...")
    stop_requested = True

def find_input_device(pa, keyword=None, min_channels=1):
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        name = info.get("name", "")
        max_in = int(info.get("maxInputChannels", 0))
        if max_in >= min_channels:
            if keyword is None or keyword.lower() in name.lower():
                return i, info
    raise RuntimeError(
        f"Could not find an input device matching '{keyword}' "
        f"with at least {min_channels} input channels."
    )

# Register signal handlers
signal.signal(signal.SIGINT, handle_stop_signal)    # Ctrl+C
signal.signal(signal.SIGTERM, handle_stop_signal)   # kill / service stop

p = pyaudio.PyAudio()
stream = None
wf = None

try:
    device_index, device_info = find_input_device(
        p,
        keyword=DEVICE_NAME_KEYWORD,
        min_channels=RESPEAKER_CHANNELS
    )
    print(f"Using input device index {device_index}: {device_info['name']}")

    audio_format = p.get_format_from_width(RESPEAKER_WIDTH)

    wf = wave.open(WAVE_OUTPUT_FILENAME, "wb")
    wf.setnchannels(RESPEAKER_CHANNELS)
    wf.setsampwidth(p.get_sample_size(audio_format))
    wf.setframerate(RESPEAKER_RATE)

    stream = p.open(
        rate=RESPEAKER_RATE,
        format=audio_format,
        channels=RESPEAKER_CHANNELS,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=CHUNK,
    )

    print("* recording (press Ctrl+C to stop and save)")

    chunks_recorded = 0
    last_report = time.time()

    # ── No time limit: runs until Ctrl+C or kill signal ──
    while not stop_requested:
        data = stream.read(CHUNK, exception_on_overflow=False)
        wf.writeframes(data)
        chunks_recorded += 1

        if time.time() - last_report >= 60:
            seconds_done = chunks_recorded * CHUNK / RESPEAKER_RATE
            print(f"Recorded about {seconds_done:.1f} seconds...")
            last_report = time.time()

    print("* recording finished")

except Exception as e:
    print(f"\nRecording error: {e}", file=sys.stderr)

finally:
    try:
        if stream is not None:
            stream.stop_stream()
            stream.close()
    except Exception:
        pass
    try:
        if wf is not None:
            wf.close()
            print(f"Audio saved as {WAVE_OUTPUT_FILENAME}")
    except Exception as e:
        print(f"Error while closing WAV file: {e}", file=sys.stderr)
    p.terminate()