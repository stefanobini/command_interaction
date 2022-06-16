from _requests.myRequestGet import MyRequestGet
from _requests.myRequestPost import MyRequestPost

address = "172.24.235.21"
port = 1026
isExit = False

# Context Broker data acquisition
selection_port = input("CB data\n" \
                       "Enter CB port : ")  # port Context Broker

selection_address_CB = input("Enter CB ip address : ")  # address ip Context Broker

requestGet = MyRequestGet(selection_address_CB, selection_port)
requestPost = MyRequestPost(selection_address_CB, selection_port)

while not isExit:
    command = input("_______________________________________________\n" \
                      "| Select command:                            |\n" \
                      "| 1      - Increase the height               |\n" \
                      "| 2      - Decrease the height               |\n" \
                      "| 3      - Bring me the screwdriver          |\n" \
                      "| 4      - Take the screwdriver              |\n" \
                      "| 5      - Bring me the windows control panel|\n" \
                      "| 6      - Take the windows control panel    |\n" \
                      "| 7      - Bring me the rearview mirror      |\n" \
                      "| 8      - Take the rearview mirror          |\n" \
                      "| 9      - Release                           |\n" \
                      "| others - Exit                              |\n" \
                      "______________________________________________\n")
    if command not in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
        print("Bye Bye!!!")
        isExit = True  # exit
    else:
        confidence = input("Confidence: ")
        requestPost.send_command(command, confidence)

        isExit = False  # continue
