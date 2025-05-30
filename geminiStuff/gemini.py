from google import genai
import speech_recognition as sr
import pyttsx3
from dotenv import load_dotenv
import os

load_dotenv()

engine = pyttsx3.init()
voices = engine.getProperty("voices")

for index, voice in enumerate(voices):
    print(f"{index}: {voice.name} - {voice.id}")

api_key = os.getenv("GENAI_API_KEY")
client = genai.Client(api_key=api_key)

with open("context.txt", "r", encoding="utf-8") as f:
    context = f.read()

# === SETUP TTS ===
engine = pyttsx3.init()

def speak(text, mood="neutral"):
    engine = pyttsx3.init()

    # Get all voices
    voices = engine.getProperty("voices")

    # Choose your preferred voice index (you can change this after testing!)
    engine.setProperty("voice", voices[1].id)  # ← Try voices[1] for "Zira" on Windows

    # Set mood-based tone
    if mood == "happy":
        engine.setProperty("rate", 190)
        engine.setProperty("volume", 1.0)
    elif mood == "sad":
        engine.setProperty("rate", 190)
        engine.setProperty("volume", 0.6)
    elif mood == "angry":
        engine.setProperty("rate", 190)
        engine.setProperty("volume", 1.0)
    elif mood == "serious":
        engine.setProperty("rate", 190)
        engine.setProperty("volume", 0.8)
    else:  # neutral/default
        engine.setProperty("rate", 190)
        engine.setProperty("volume", 0.9)

    # Speak the response
    engine.say(text)
    engine.runAndWait()

# === FUNCTION TO RECOGNIZE SPEECH ===
def recognize_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("testing...")
        audio = recognizer.listen(source)
    try:
        return recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        return "Scouldn't understand that."
    except sr.RequestError:
        return "didn't work"

def detect_mood(text):
    text = text.lower()
    if any(word in text for word in ["great!", "amazing", "awesome", "yay", "hilarious", "haha"]):
        return "happy"
    elif any(word in text for word in ["sorry", "unfortunately", "sad", "apologize", "regret"]):
        return "sad"
    elif any(word in text for word in ["angry", "mad", "furious", "why would you", "seriously?"]):
        return "angry"
    elif any(word in text for word in ["however", "therefore", "must", "note", "important"]):
        return "serious"
    else:
        return "neutral"

conversation = ""

for i in range(10):
    print(f"\nRound {i+1} — speak")
    user_input = recognize_speech()
    print(f"You said: {user_input}")

    # Append your new question to the running conversation
    conversation += f"\nUser: {user_input}"

    # Build the full prompt with context + conversation
    full_prompt = f"""You are having a conversation with a helpful assistant.
This is the background context document:

{context}

Here is the current conversation:
{conversation}

Assistant:"""

    # Call Gemini
    response = client.models.generate_content(
        model="gemini-2.0-flash",
        contents=full_prompt
    )

    response_text = response.text.strip()
    print("Gemini says:")
    print(response_text)

    # Add Gemini's reply to the conversation for memory
    conversation += f"\nAssistant: {response_text}"

    # Speak it
    mood = detect_mood(response_text)
    print(f"Detected mood: {mood}")
    speak(response_text, mood)

