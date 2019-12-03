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
  name : '/animal_file',
  messageType : 'uchile_msgs/AnimalCard'
});


var animal_info = new ROSLIB.Topic({
  ros : ros,
  name : '/animal_info',
  messageType : 'uchile_msgs/AnimalCard'
});

function info_publisher(){
  console.log("Publishing!");
  var nombre = document.getElementById("nombre");
  var locacion = document.getElementById("locacion");
  var restantes = document.getElementById("restantes");
  var datos_extra = document.getElementById("datos_extra");
  var autor = document.getElementById("autor");
  var img = document.getElementById("foto");
  var info = new ROSLIB.Message({
    name : nombre.value,
    location : locacion.value,
    left_species : restantes.value,
    pic : img.src,
    extra_info : datos_extra.value,
    author : autor.value
  });
  animal_info.publish(info);
}

// animal_info.publish("uwu","uwu","uwu","uwu","uwu","uwu");

animal_file.subscribe(function(message) {
  var name = message.name;
  console.log(name)
  var nombre = document.getElementById("nombre");
  var locacion = document.getElementById("locacion");
  var restantes = document.getElementById("restantes");
  var datos_extra = document.getElementById("datos_extra");
  var autor = document.getElementById("autor");
  nombre.value = name;
  locacion.value = message.location;
  restantes.value = message.left_species;
  datos_extra.value = message.extra_info;
  autor.value = message.author;
  var img = document.getElementById("foto");
  img.src = message.pic;
});
var bg = document.getElementById("alpha")
var button = document.getElementById("starto");
button.style.display = "none";
bg.style.display = "none";

}