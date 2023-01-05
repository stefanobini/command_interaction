function updateView(toShowId) {
    tabs = document.querySelectorAll(".tab");
    for(i=0;i<tabs.length;i++){
        button = document.getElementById("btn_"+tabs[i].id);
        if(tabs[i].id === toShowId) {
            tabs[i].style.display = "block";
            button.classList.add("tab-active");
            button.classList.remove("view-cng");
        } else {
            tabs[i].style.display = "none";
            button.classList.remove("tab-active");
            button.classList.add("view-cng");
        } 
    }    
}

function addMessage(message, type) {
    canvas = document.getElementById("msg-canvas");
    cont_node = document.createElement("div");
    cont_node.classList.add("msg");
    cont_node.classList.add(type);
    p_node = document.createElement("p");
    text_node = document.createTextNode(message);
    p_node.appendChild(text_node)
    cont_node.appendChild(p_node);
    canvas.appendChild(cont_node, canvas.firstChild);
    if(canvas.childElementCount > 2) canvas.removeChild(canvas.children[0]);
}

function addUserMessage(message) {
    addMessage(message, "usr-msg");
}

function addPepperMessage(message) {
    addMessage(message, "ppr-msg");
}

function lastReceivedMessage(message) {
    canvas = document.querySelector("#speech_popup > p");
    canvas.innerText = message;
}

function showPopup(message) {
    if(message) {
        document.querySelector("#popup > p").innerText = message;
    }
    document.getElementById("popup").style.display = "block";
}

function hidePopup() {
    document.getElementById("popup").style.display = "none";
}

function updateLastText(colorClass, nodeName, message) {
    node = document.createElement("p");
    node.classList.add(colorClass);
    textnode = document.createTextNode("["+nodeName+"] " + message);
    node.appendChild(textnode);
    console = document.getElementById("console");
    console.appendChild(node);
    if(console.childElementCount >= 15) console.removeChild(console.children[0]);
}