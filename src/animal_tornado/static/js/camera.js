      var RECORD_ON ="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAMAAAAp4XiDAAAASFBMVEUAAAAAAADMzMz/AABmZmaZmZnMAABmAACZAAAzAAAzMzP/MzPMZmaZMzPMMzP/MwD/////Zmb/ZgBmMzPMmZmZZmb/mQD/mZlxvxKlAAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAAAHdElNRQffCBIMMyVTqZicAAACK0lEQVRIx43W63aDIAwA4KJJhVjrVtfu/d90kASIFlzzp5fDd5KAgJdLJwa8fB6YYojBXz4EQw1EgNPxAGDHqzpDBqDV0EN5DDoToIhagkSAOwb2DIEBRLEWINJf2DQspCQ7AwiUy0M69pEFyCR/xZCl4cTE/CgQtYoU2xZC2J6YiXNHA3ai4uApRwjaj+RBs+JWpKHLcrstiyAz3WjFUFJMk4/jf39+fqPyCdXJLmmwzi4L7+8a8avPhlwtzSQRcY0x3+9z+mSz8bSY0jRJWoEirtdxlM+IluczEXA7In+FbVm8jJ9jqIpmC2nBSjc6XXFZwvS83XwC4/wdI5pRDLczUE6TSJoPisRrDhNCkqFSmTY/pN6ZpIo05pguGZk1qawSKL3PxnznNIGHkDSjK1/q4tZNnlFI0CdNCeiTooRjXVfTTV5PJnggWTxejzNCDfJ6PdYWGd6zXHOWR7ewXi/jOrYJ8PbCvFH8cSGFlBmjQoZC/Dup68LbjAmWR39fWaOu1MRFCNg0rWesrCQTU1njuaxPZW1lX1neYRXwvgy1rkLQGN2WZVOWJHFPDZBPMUTXMVZAvZ+gnolqfBlvT6VEymGJelzryedL7M4+MFdTvIwAnDn8JnPCupbgO7KaeijHLz1xNKzseBVweTPketEQQgDagLAhssFWIl6P5j3OpK5QzgB8lXVufiFoFkl/n71fZIPlDeYsxa64+l7yH1BFUAv6ANgZ7wz/A295IO9de7kCAAAAAElFTkSuQmCC";
      var RECORD_OFF = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyBAMAAADsEZWCAAAAFVBMVEVvcm0AAAAzMzNmZmbMzMyZmZn///8pcdebAAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAAAHdElNRQffCBINBR5qfIr6AAABv0lEQVQ4y22UQZaCMAyGffo8QED3Q9E9EDmAYzrrgZIbzJv7H2GStFLLmA22H/+fWNLsdi/hd2+DeSHP/A6Qhg9duX/AhVJskAGPoyLsNmACjVrQkMFe3u0hxUKPj5W0BqoWh0YeSxbtb2o1xvyNGPqnCMkLYPr6/SEOABcKqeKbF8mV2UE1KgHysbyDStaQTCKKR8GpYHDoIJVuJ3hcyJZVK1ZanFbuv5XQIypaRBycoupkdkx3kzhBg3NOMl2j3UJ9LS+6NRCufSLyq9KtqBFVPWmi4+KBoUbZkQoUiMHniZTcYYYqmylpjUgBdyVohu2s5HxOpDdiSWbNA3UlZR/lBBqw/CYyIkenRL9ZqjfVFgln0vJckiaTkIkvNYXba57WZRLku03xPHPo2sj9DamVLFuiFZ2pE2IfbkNOQnY32hBdXrR7RoLSzhrhYWQqRNYj/NB281MhsrZiVGJtuYpiw7E16RiajGIrYrxdGHooo0ZMdxQ3BJ8XErHfSrrnxcamlOB65Qu/K+bbLWTOXrxKDKU/JU3I/DoQDsjMgwmYyyFyGNkGj4ykgOXgET8bSf+AqAZ1DIhvBhwivu7/ATYxlvOTH50RAAAAAElFTkSuQmCC";

      var alpha, valpha, z;
      var beta, vbeta, x;
      var gamma, vgamma, y;

      var cameraTimer, imuTimer;

// setup event handler to capture the orientation event and store the most recent data in a variable

      if (window.DeviceOrientationEvent) {
        // Listen for the deviceorientation event and handle the raw data
        window.addEventListener('deviceorientation', function(eventData) {
          // gamma is the left-to-right tilt in degrees, where right is positive
          gamma = eventData.gamma;
          
          // beta is the front-to-back tilt in degrees, where front is positive
          beta = eventData.beta;
          
          // alpha is the compass direction the device is facing in degrees
          alpha = eventData.alpha

          }, false);
        };

// setup event handler to capture the acceleration event and store the most recent data in a variable

        if (window.DeviceMotionEvent) {
          window.addEventListener('devicemotion', deviceMotionHandler, false);
        } else {
          window.alert("acceleration measurements Not supported.");
        }

        function deviceMotionHandler(eventData) {
          // Grab the acceleration from the results
          var acceleration = eventData.acceleration;
          x = acceleration.x;
          y = acceleration.y;
          z = acceleration.z;

          // Grab the rotation rate from the results
          var rotation = eventData.rotationRate;
          vgamma = rotation.gamma;  
          vbeta = rotation.beta;
          valpha = rotation.alpha;
}


// setup connection to the ROS server and prepare the topic
  var ros = new ROSLIB.Ros();

  ros.on('connection', function() { console.log('Connected to websocket server.');});

  ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });

  ros.on('close', function() { console.log('Connection to websocket server closed.');});

  var imageTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/image/compressed',
    messageType : 'sensor_msgs/CompressedImage'
  });

  var imuTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/gyro',
    messageType : 'sensor_msgs/Imu'
  });