function init() {
    if ("WebSocket" in window) {
        // log function
        log = function(data){
            $("div#terminal").prepend("</br>" +data);
            console.log(data);
        };
        $("div#message_details").show();
        $("#open").click(function(evt) {
            evt.preventDefault();

            // var host = $("#host").val();
            // var port = $("#port").val();
            // var uri = $("#uri").val();

            // create websocket instance
            //ws = new WebSocket("ws://" + host + ":" + port + uri);
            ws = new WebSocket("ws://localhost:8888")
            // Handle incoming websocket message callback
            ws.onmessage = function(evt) {
                log("Message Received: " + evt.data)
                //alert("message received: " + evt.data);
            };

            // Close Websocket callback
            ws.onclose = function(evt) {
                log("***Connection Closed***");
                alert("Connection close"); 
                $("#uri").css("background",  "#ff0000");
                //$("div#message_details").empty();
            };

            // Open Websocket callback
            ws.onopen = function(evt) { 
                $("#uri").css("background", "#00ff00");
                //$("div#message_details").show();
                log("***Connection Opened***");
            };
        });

        // Send websocket message function
        $("#send_run").click(function(evt) {
            log("Launching ROSCORE");
            ws.send("run_ROSCORE");
        });
        $("#send_kill").click(function(evt) {
            log("Killing ROSCORE");
            ws.send("kill_ROSCORE");
        });
        // $("#launch_mapper").click(function(evt) {
        //     log("launching MAPPER");
        //     ws.send("launch_mapper");
        // });
        // $("#run_AQ").click(function(evt) {
        //     log("Running AnswerQuestion");
        //     ws.send("AnswerQuestion");
        // });
        $("#spr_launcher").click(function(evt) {
            log("Starting launcher for SPR");
            ws.send("spr_launcher");
        });
        $("#spr_test").click(function(evt) {
            log("SPR");
            ws.send("spr_test");
        });
        $("#spr_kill").click(function(evt) {
            log("Killing SPR");
            ws.send("spr_kill");
        });
        // ws = new WebSocket("ws://127.0.0.1:8888/ws/");
        // ws.onopen = function() {
        //     console.log("Connection is opened");
        // }
        // ws.onclose = function() {
        //     console.log("Connection is closed");
        // }
        // ws.onmessage = function(msg) {
        //     document.getElementById("display").innerHTML = msg.data;
        // }    
    } else {
        console.log('Your browser doenst support WebSocket!')
    }
}
function send() {
    ws.send(document.getElementById("txt").value);
}