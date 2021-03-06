Pose = function(ros, name, userActionClient) {
  var that = this;
  this.name = name;

  function makeUserMessage(commandArg, nameArg) {
    userMessage = new ROSLIB.ServiceRequest({
      command: commandArg,
      name: nameArg
    });
    userActionClient.callService(userMessage, function(result){});
  } 

  function handleGoTo() {
    makeUserMessage("goto", name)
    console.log('Go to ' + name + ' clicked.');
  }

  function handleDelete() {
    makeUserMessage("delete", name)
    console.log('Delete ' + name + ' clicked.');
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);
    return node;
  }

}
