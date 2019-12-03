// Connecting to ROS
// -----------------
function ws_connect(){
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

  var animal_file = new ROSLIB.Topic({
  ros : ros,
  name : '/SkeletonWeb',
  messageType : 'uchile_msgs/SkeletonWeb'
});

animal_file.subscribe(function(message) {
  var x_pos = message.Hand[0];
  var y_pos = message.Hand[1];
  var xpos = (x_pos-100);
  var ypos = (y_pos-100);
  var t = message.Time;
  var zone = message.Zone;
  var count = document.getElementById("count");
  console.log(x_pos+", "+y_pos)
  // count.innerHTML = t;
  document.getElementById("za_hando").style.left = xpos+"px";
  document.getElementById("za_hando").style.top = ypos+"px";
  var img = document.getElementById("hando");
  img.src = "../static/img/hand/"+t+".png";
  var mapita = document.getElementById("mapa");
  mapita.src = "../static/img/map/"+zone+".png";
});

var bg = document.getElementById("alpha")
var button = document.getElementById("starto");
button.style.display = "none";
bg.style.display = "none";

}