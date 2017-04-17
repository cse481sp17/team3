Head = function(ros) {
  // HTML elements
  // alert("AM I WORKING?");
  var desiredHeadPan = document.querySelector('#desiredHeadPan');
  var desiredHeadTilt = document.querySelector('#desiredHeadTilt');

  var headPanSlider = document.querySelector('#headPanSlider');
  var headTiltSlider = document.querySelector('#headTiltSlider');

  var headButton = document.querySelector('#headButton');

  var that = this;

  var setTorsoClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_head',
    serviceType: 'web_teleop/SetHead'
  });

  // Listen to torso height from the joint_state_republisher.

  /*
  var listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/torso_lift_joint',
    messageType: 'std_msgs/Float64'
  });


  listener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var height = message.data;

    // Note the noise in the data. You can smooth it out using this line of code.
    // height = Math.round(height*1000) / 1000
    torsoHeight.textContent = height;
  });
  */

  // Initialize slider.
  var desiredHeadPan = 0.1;
  desiredHeadPan.textContent = desiredHeadPan;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  headPanSlider.value = desiredHeadPan;

  // Initialize slider.
  var desiredHeadTilt = 0.1;
  desiredHeadTilt.textContent = desiredHeadTilt;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  headTiltSlider.value = desiredHeadTilt;

  // Update desiredHeight when slider moves.
  headPanSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredHeadPan = headPanSlider.value;
    // Update the desired torso height display.
    desiredHeadPan.textContent = desiredHeadPan;
  });

  // Update desiredHeight when slider moves.
  headTiltSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredHeadTilt = headTiltSlider.value;
    // Update the desired torso height display.
    desiredHeadTilt.textContent = desiredHeadTilt;
  });

  // Method to set the height.
  this.setHead = function(pan, tilt) {
    console.log(pan);
    console.log(tilt);

    pan = parseFloat(pan);
    tilt = parseFloat(tilt);

    console.log(pan);
    console.log(tilt);

    var request = new ROSLIB.ServiceRequest({
      pan: pan,
      tilt: tilt
    });
    setTorsoClient.callService(request);
  };

  // Set the height when the button is clicked.
  headButton.addEventListener('click', function() {
    // console.log("SUCK M YASS");
    // alert("PLZ WORK");
    that.setHead(desiredHeadPan, desiredHeadTilt);
  });
}
