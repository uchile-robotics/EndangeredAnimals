################################################
######### INTERFAZ DE LA LAMINA ################
################################################

0.- Instalar rosbridge_server
1.- Lanzar "rosbridge_websocket.launch"

	>     $ roslaunch rosbridge_server rosbridge_websocket.launch   

Por defecto, abre el WebSocket en "//localhost:9090"
Si el servidor comienza correctamente se deberia mostrar lo siguiente:

	>     [INFO] [/rosbridge_websocket]: Rosbridge WebSocket server started on port 9090

2.- Abrir el archivo "templates/index.html" en navegador de preferencia (Que no sea Internet Explorer!!)
Al abrirlo, este importa el script "static/js/rostopic_listener.js" que se conecta al WebSocket creado en
el paso 1, y se suscribe a al topico "/animal_file" y recibe mensajes del tipo "uchile_msgs/AnimalCard"
Si la subscripción se realizo correctamente, se deberia ver lo siguiente en la terminal donde se lanzo "rosbridge_websocket.launch":

	>     [INFO] [/rosbridge_websocket]: Client connected.  1 clients total.
	>     [INFO] [/rosbridge_websocket]: [Client 0] Subscribed to /animal_file

AnimalCard tiene la siguiente estructura:

	string name
	string location
	int64 left_species
	string pic
	string extra_info
	string author

El topico y el mensaje al que se suscribe se pueden modificar en "static/js/rostopic_listener.js".

Se puede probar que la pagina esta recibiendo los datos correctamente mediante:
	1.- Ejecutar en la terminar:

		>     $ rostopic pub /animal_file uchile_msgs/AnimalCard "{name: 'uwu', location: 'iwi', left_species: 420, pic: '', extra_info: 'twt', author: 'owo'}"

	2.- Correr el script "test.py":

		>     $ python test.py

NOTAS:
	-> La lamina muestra por defecto la imagen de una rana, siempre lo va a mostrar independiente del dato enviado en "message.pic", esto hay que cambiarlo luego.

################################################
######### INTERFAZ DEL MAPA ####################
################################################

1.- Lanzar "rosbridge_websocket.launch"

	>     $ roslaunch rosbridge_server rosbridge_websocket.launch   

Por defecto, abre el WebSocket en "//localhost:9090"
Si el servidor comienza correctamente se deberia mostrar lo siguiente:

	>     [INFO] [/rosbridge_websocket]: Rosbridge WebSocket server started on port 9090

2.- Abrir el archivo "templates/hand.html" en navegador de preferencia (Que no sea Internet Explorer!!)
Al abrirlo, este importa el script "static/js/rostopic_position.js" que se conecta al WebSocket creado en
el paso 1, y se suscribe a al topico "/SkeletonWeb" y recibe mensajes del tipo "uchile_msgs/SkeletonWeb"
Si la subscripción se realizo correctamente, se deberia ver lo siguiente en la terminal donde se lanzo "rosbridge_websocket.launch":

	>     [INFO] [/rosbridge_websocket]: Client connected.  1 clients total.
	>     [INFO] [/rosbridge_websocket]: [Client 0] Subscribed to /SkeletonWeb

SkeletonWeb tiene la siguiente estructura:

	int32[] X_skeleton
	int32[] Y_skeleton
	int32[] Hand
	int32 Zone
	int32 Time

El topico y el mensaje al que se suscribe se pueden modificar en "static/js/rostopic_position.js".