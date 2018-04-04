
update_element = (data) ->
  `let e =  document.getElementById(data['id']);
   if (e) {
     if (data['component'] === 'AssetRemoved') {
       let p = e.parentNode;
       p.parentNode.removeChild(p);
     } else {
       $("#" + data['id']).html(data['value']);
     }
   } else {
     if (data['component'] === 'Asset') {
       $("#" + data['name'] + '_assets').append('<div><h3>' + data['item'] +
         ' ' + data['raw_id'] + '</h3><pre id="' + data['id'] +
         '">' + data['value'] + '</pre></div>');
     }
   }`
  return ''      

App.mtc_events = App.cable.subscriptions.create "MtcEventsChannel",
  connected: ->
    # Called when the subscription is ready for use on the server

  disconnected: ->
    # Called when the subscription has been terminated by the server

  received: (data) ->
    # Called when there's incoming data on the websocket for this channel
    update_element(data)
