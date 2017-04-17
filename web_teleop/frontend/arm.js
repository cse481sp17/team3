Arm = function(ros) {
  // HTML elements

  var currentArm = document.querySelector('#currentArm');
  var desiredArm = document.querySelector('#desiredArm');
  var armButton = document.querySelector('#armButton');

  var that = this;

  var setTorsoClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_arm',
    serviceType: 'web_teleop/SetArm'
  });

  // Listen to torso height from the joint_state_republisher.
  /* var listener = new ROSLIB.Topic({
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
  }); */

  // Method to set the height.
  this.setArm = function(arm) {
    console.log(arm);

    arm = arm.split(' ').map(function(x) {return parseFloat(x)});

    console.log(arm);

    var request = new ROSLIB.ServiceRequest({
      positions: arm
    });

    setTorsoClient.callService(request);
  };

  // Set the height when the button is clicked.
  armButton.addEventListener('click', function() {
    // console.log("SUCK M YASS");
    // alert("PLZ WORK");
    that.setArm(desiredArm.value);
  });
}
