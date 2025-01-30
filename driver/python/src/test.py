import limu_py

foo = limu_py.ToF.tof320("10.10.31.180", "50660")

def handleFrame(frame):
    print("got frame")

foo.subscribeFrame(handleFrame)
foo.streamDistance()

while True:
    user_input = input("Enter a command (or 'quit' to exit): ")
    if user_input == "quit":
        break
    # Process the user input
    print("You entered:", user_input)