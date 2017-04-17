Gripper = function(ros) {
  // HTML elements
  var gripperButton = document.querySelector('#gripperToggle');

  var that = this;

  var setTorsoClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_gripper',
    serviceType: 'web_teleop/SetGripper'
  });

  // Method to set the height.
  this.setGripper = function(gripper_val) {

    var request = new ROSLIB.ServiceRequest({
      todo: gripper_val
    });

    setTorsoClient.callService(request);
  };

  // Set the height when the button is clicked.
  gripperButton.addEventListener('click', function() {
    that.setGripper(gripperButton.checked);
  });
}
