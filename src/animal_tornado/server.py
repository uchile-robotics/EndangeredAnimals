#!/usr/bin/env bash

import tornado.ioloop
import tornado.web
import tornado.websocket
import socket
import os.path

from tornado.options import define, options, parse_command_line

define("port", default=8888, type=int)

# Carpetas para archivos statics e html's 
settings = dict(
		template_path=os.path.join(os.path.dirname(__file__), "templates"),
		static_path=os.path.join(os.path.dirname(__file__), "static"),
		debug=True

)

# Clase que renderiza el index (html con el websocket)
class IndexHandler(tornado.web.RequestHandler):
	def get(self):
		self.render("index.html")

class MapHandler(tornado.web.RequestHandler):
	def get(self):
		self.render("map.html")

class PicsHandler(tornado.web.RequestHandler):
	def get(self):
		self.render("pics.html")

class InitHandler(tornado.web.RequestHandler):
	def get(self):
		self.render("first.html")

# Url's 
app = tornado.web.Application([
	(r'/', IndexHandler),
	(r'/map', MapHandler),
	(r'/pics', PicsHandler),
	(r'/init', InitHandler),
], **settings)

# Corre el servidor
if __name__ == '__main__':
	print 'running'
	myIP = socket.gethostbyname(socket.gethostname())
	print '*** Websocket Server Started at %s***' % myIP
	app.listen(options.port)
	tornado.ioloop.IOLoop.instance().start()