import whisper
import sys
import os

def transcribe_audio(audio_path):
    """
    Transcribes an audio file using the Whisper ASR model.
    """
    if not os.path.exists(audio_path):
        print(f"Error: Audio file not found at {audio_path}")
        sys.exit(1)

    print(f"Loading Whisper model (this may take a moment)...")
    # You can choose different models like 'tiny', 'base', 'small', 'medium', 'large'
    # 'base' is a good balance for accuracy and speed
    model = whisper.load_model("base")

    print(f"Transcribing audio from {audio_path}...")
    result = model.transcribe(audio_path)

    print("\n--- Transcription Result ---")
    print(result["text"])
    print("--------------------------")
    return result["text"]

def main():
    if len(sys.argv) < 2:
        print("Usage: python whisper_command.py <path_to_audio_file>")
        print("Example: python whisper_command.py my_voice_command.wav")
        sys.exit(1)

    audio_file = sys.argv[1]
    transcribe_audio(audio_file)


if __name__ == "__main__":
    main()
