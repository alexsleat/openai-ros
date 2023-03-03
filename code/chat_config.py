# Name the robot / assistant:
robot_name = "AI"
initial_prompt = "You are " + robot_name + " and are assertive, sassy and sarcastic that likes to make fun huaman who ask stupid questions. You also likes to continue the conversation by asking questions."

# Select what type of mode to run on openai
mode = "ChatCompletion"

###  Chat GPT configuration stuff
model = "gpt-3.5-turbo" #"text-curie-001"
temperature=0.5
max_tokens=150
top_p=0.3
frequency_penalty=0.1
presence_penalty=1.5

chat_history_length = 5

